% TrainSegmentation.m
% =========================================================================
% Fine-tune DeepLabv3+ on IDD (India Driving Dataset)
% Task: Binary road segmentation — drivable vs non-drivable
% Uses ResNet-18 encoder pretrained on ImageNet
% =========================================================================
% OUTPUT: ml/models/deeplab_net.mat
% =========================================================================

clc; clear;
addpath(genpath('.'));

fprintf('=== DeepLabv3+ Fine-Tuning: Road Segmentation (IDD) ===\n\n');

%% --- 1. Configuration ---
INPUT_SIZE  = [360 480 3];   % H x W x C (IDD standard)
NUM_CLASSES = 2;             % road | background
classNames  = ["road", "background"];
labelIDs    = {[0 100 200], [50 150 250]};  % approximate RGB label colors

dataDir = fullfile(fileparts(mfilename('fullpath')), 'data', 'idd');

%% --- 2. Load IDD Image & Label Data ---
imgFiles   = dir(fullfile(dataDir, '**', '*.jpg'));
labelFiles = dir(fullfile(dataDir, '**', '*.png'));  % segmentation masks

if isempty(imgFiles)
    fprintf('[WARNING] No IDD images found in %s\n', dataDir);
    fprintf('Creating synthetic road segmentation data...\n');
    [imds, pxds] = CreateSyntheticSegData(INPUT_SIZE, NUM_CLASSES, classNames);
else
    fprintf('Found %d images and %d label masks.\n', numel(imgFiles), numel(labelFiles));
    imgPaths   = fullfile({imgFiles.folder}, {imgFiles.name});
    labelPaths = fullfile({labelFiles.folder}, {labelFiles.name});

    imds  = imageDatastore(imgPaths);
    pxds  = pixelLabelDatastore(labelPaths, classNames, labelIDs);
end

% Combined training datastore
trainingDS = combine(imds, pxds);

%% --- 3. Build DeepLabv3+ with ResNet-18 Encoder (pretrained) ---
fprintf('\nBuilding DeepLabv3+ with pretrained ResNet-18 encoder...\n');

try
    lgraph = deeplabv3plusLayers(INPUT_SIZE, NUM_CLASSES, 'resnet18');
    fprintf('  DeepLabv3+ graph created: %d layers\n', numel(lgraph.Layers));
catch e
    fprintf('[ERROR] deeplabv3plusLayers failed: %s\n', e.message);
    fprintf('Ensure Computer Vision Toolbox + Deep Learning Toolbox are licensed.\n');
    return;
end

%% --- 4. Training Options ---
% Class weights: road typically underrepresented on unmarked roads
tbl         = countEachLabel(pxds);
imageFreq   = tbl.PixelCount / sum(tbl.PixelCount);
classWeight = 1 ./ imageFreq;
classWeight = classWeight / sum(classWeight);

fprintf('  Class weights: road=%.2f, background=%.2f\n', classWeight(1), classWeight(2));

options = trainingOptions('adam', ...
    'InitialLearnRate',    1e-4, ...
    'MaxEpochs',           20, ...
    'MiniBatchSize',       4, ...
    'LearnRateSchedule',   'piecewise', ...
    'LearnRateDropFactor', 0.5, ...
    'LearnRateDropPeriod', 10, ...
    'Shuffle',             'every-epoch', ...
    'VerboseFrequency',    10, ...
    'Plots',               'training-progress', ...
    'CheckpointPath',      fullfile(fileparts(mfilename('fullpath')), 'models'));

%% --- 5. Fine-Tune ---
fprintf('\nStarting fine-tuning of DeepLabv3+...\n');

try
    [net, info] = trainNetwork(trainingDS, lgraph, options);

    modelPath = fullfile(fileparts(mfilename('fullpath')), 'models', 'deeplab_net.mat');
    save(modelPath, 'net', 'classNames', 'INPUT_SIZE');
    fprintf('\n[SUCCESS] DeepLabv3+ saved: %s\n', modelPath);

    % Quick eval on first image
    testImg  = readimage(imds, 1);
    testImg  = imresize(testImg, INPUT_SIZE(1:2));
    predMask = semanticseg(testImg, net);
    roadFrac = mean(predMask == "road", 'all');
    fprintf('  Sample road fraction: %.1f%%\n', roadFrac*100);

catch e
    fprintf('\n[ERROR] Segmentation training failed: %s\n', e.message);
    net = [];
    modelPath = fullfile(fileparts(mfilename('fullpath')), 'models', 'deeplab_net.mat');
    save(modelPath, 'net', 'classNames', 'INPUT_SIZE');
end

fprintf('=== TrainSegmentation Complete ===\n');


%% --- Helper: Synthetic Segmentation Data ---
function [imds, pxds] = CreateSyntheticSegData(inputSize, ~, classNames)
    tmpDir  = fullfile(tempdir, 'sih_seg_train');
    imgDir  = fullfile(tmpDir, 'images');
    lblDir  = fullfile(tmpDir, 'labels');
    mkdir(imgDir); mkdir(lblDir);

    N = 20;
    imgPaths = cell(N,1);
    lblPaths = cell(N,1);
    for i = 1:N
        img  = uint8(rand(inputSize)*200 + 55);
        % Road = bottom 40% of image (typical road view)
        lbl  = zeros(inputSize(1), inputSize(2), 3, 'uint8');
        roadStart = round(inputSize(1)*0.6);
        lbl(roadStart:end,:,:) = repmat(reshape([0 100 200],[1 1 3]), ...
                                        inputSize(1)-roadStart+1, inputSize(2));
        lbl(1:roadStart-1,:,:) = repmat(reshape([50 150 250],[1 1 3]), ...
                                        roadStart-1, inputSize(2));
        ip = fullfile(imgDir, sprintf('img_%03d.jpg',i));
        lp = fullfile(lblDir, sprintf('lbl_%03d.png',i));
        imwrite(img, ip);
        imwrite(lbl, lp);
        imgPaths{i} = ip;
        lblPaths{i} = lp;
    end

    labelIDs = {[0 100 200],[50 150 250]};
    imds = imageDatastore(imgPaths);
    pxds = pixelLabelDatastore(lblPaths, classNames, labelIDs);
end
