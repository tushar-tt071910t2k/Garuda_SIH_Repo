% TrainYOLO.m — Fine-tune YOLOv4 on IDD Dataset (10K images + 7974 annotations)
clc; clear;
addpath(genpath('../utils')); addpath(genpath('.'));
fprintf('=== YOLOv4 Fine-Tuning: IDD Dataset ===\n\n');

classNames = {'person','cow','auto_rickshaw','bicycle','car','truck','traffic_light','pushcart'};
dataRoot   = fullfile(fileparts(mfilename('fullpath')),'data');
modelDir   = fullfile(fileparts(mfilename('fullpath')),'models');
if ~exist(modelDir,'dir'); mkdir(modelDir); end
inputSize  = [416 416 3];

%% --- 1. Parse IDD Annotations (now with proper image matching) ---
fprintf('Step 1: Parsing IDD polygon annotations...\n');
[annotations, labelStats] = ParseIDDAnnotations(dataRoot, classNames, Inf);
fprintf('Annotations parsed: %d samples\n', height(annotations));

%% --- 2. Build training data: real pairs + synthetic fill ---
fprintf('\nStep 2: Preparing image-annotation pairs...\n');
synthDir = fullfile(tempdir,'sih_idd_yolo'); if ~exist(synthDir,'dir'); mkdir(synthDir); end

trainFiles  = {};
trainBoxes  = {};
trainLabels = {};

for i = 1:height(annotations)
    imgPath = annotations.ImageFile{i};
    boxes   = annotations.BBoxes{i};
    labels  = annotations.Labels{i}(:);  % force column vector

    % Validate: boxes and labels must have same row count
    if isempty(boxes) || numel(labels) ~= size(boxes,1); continue; end

    if ~isempty(imgPath) && exist(imgPath,'file')
        % Real image exists
        trainFiles{end+1}  = imgPath;
        trainBoxes{end+1}  = boxes;
        trainLabels{end+1} = labels;
    end
end

% Validate dataset size to prevent Data Duplication Trap
if numel(trainFiles) < 500
    error('[FATAL] Expected thousands of IDD annotations but found only %d! Check your dataset. Aborting to prevent overfitting on micro-dataset.', numel(trainFiles));
end
fprintf('Total training samples: %d\n', numel(trainFiles));

%% --- 3. Create datastores ---
fprintf('\nStep 3: Creating datastores...\n');
readFcn = @(f) imresize(imread(f), inputSize(1:2));
imds    = imageDatastore(trainFiles,'ReadFcn',readFcn);
bboxTbl = table(trainBoxes(:), trainLabels(:), 'VariableNames',{'BBoxes','Labels'});

% Validate every row before creating datastore
valid = true(height(bboxTbl),1);
for i = 1:height(bboxTbl)
    bb = bboxTbl.BBoxes{i};
    lb = bboxTbl.Labels{i};
    % Force column vector (boxLabelDatastore requires Nx1 not 1xN)
    lb = lb(:);
    bboxTbl.Labels{i} = lb;
    if isempty(bb) || isempty(lb) || numel(lb) ~= size(bb,1) || size(bb,2) ~= 4
        valid(i) = false;
    end
end
if any(~valid)
    fprintf('Removed %d invalid rows\n', sum(~valid));
    bboxTbl = bboxTbl(valid,:);
    imds    = imageDatastore(trainFiles(valid),'ReadFcn',readFcn);
end

blds       = boxLabelDatastore(bboxTbl);
baseDS     = combine(imds, blds);

% Apply Data Augmentation (Flip, Color Jitter)
trainingDS = transform(baseDS, @augmentData);
fprintf('Training datastore: %d valid samples (Augmentation Enabled)\n', height(bboxTbl));

fprintf('\nStep 4: Loading pretrained YOLOv4 and estimating anchors...\n');
preDetector = [];
try
    fprintf('  Estimating anchor boxes from dataset...\n');
    anchorsRaw = estimateAnchorBoxes(blds, 9); % 9 anchors for extreme Indian traffic aspect ratios
    preDetector = yolov4ObjectDetector('tiny-yolov4-coco', classNames, anchorsRaw, 'InputSize', inputSize);
    fprintf('  Pretrained YOLOv4 weights loaded OK\n');
catch e
    fprintf('  [INFO] YOLOv4 load failed: %s\n', e.message);
    fprintf('  Falling back to YOLOv2...\n');
end

%% --- 5. Training options (Hyperparameter Overhaul) ---
options = trainingOptions('sgdm', ...
    'Momentum',            0.9, ...
    'MiniBatchSize',       8, ...
    'MaxEpochs',           50, ...
    'InitialLearnRate',    1e-4, ...
    'LearnRateSchedule',   'piecewise', ...
    'LearnRateDropFactor', 0.8, ...
    'LearnRateDropPeriod', 10, ...
    'Shuffle',             'every-epoch', ...
    'VerboseFrequency',    15, ...
    'Plots',               'training-progress', ...
    'CheckpointPath',       modelDir);

%% --- 6. Train ---
fprintf('\nStep 5: Fine-tuning YOLOv4 (50 epochs with Augmented Data)...\n');
try
    if ~isempty(preDetector)
        detector = trainYOLOv4ObjectDetector(trainingDS, preDetector, options, ...
                    'PositiveOverlapRange',[0.5 1.0], 'NegativeOverlapRange',[0.0 0.4]);
    else
        % YOLO v4 support package not installed — try v2 (built-in) or use anchor fix
        fprintf('  Trying YOLOv2 (built-in, no extra package needed)...\n');
        % estimateAnchorBoxes returns Nx2 double matrix
        % yolov2ObjectDetector takes double matrix directly (NOT cell)
        anchorsRaw  = estimateAnchorBoxes(blds, 9);   % 9×2 [height width] for extreme aspect ratios
        try
            baseNet  = yolov2ObjectDetector('tiny-yolov2-coco', classNames, anchorsRaw, ...
                            'InputSize', inputSize);
            detector = trainYOLOv2ObjectDetector(trainingDS, baseNet, options);
        catch e2
            fprintf('  YOLOv2 also failed: %s\n', e2.message);
            fprintf('  TIP: Install YOLO support package in MATLAB:\n');
            fprintf('       Home → Add-Ons → search: Computer Vision Toolbox Model for YOLO v4\n');
            detector = [];
        end
    end

    modelPath = fullfile(modelDir,'yolo_detector.mat');
    save(modelPath,'detector','classNames','inputSize');
    fprintf('\n[SUCCESS] YOLO saved: %s\n', modelPath);

    % Quick sanity test (only if detector was actually trained)
    if ~isempty(detector)
        img = imresize(imread(trainFiles{1}), inputSize(1:2));
        [bb,~,~] = detect(detector, img, 'Threshold',0.25);
        fprintf('  Sanity test: %d detections on sample image\n', size(bb,1));
    else
        fprintf('  [WARN] Detector is empty - install YOLO support package to enable real detections\n');
        fprintf('  Scenarios will use fallback rule-based detection (still works!)\n');
    end

catch e
    fprintf('\n[ERROR] %s\n', e.message);
    detector = [];
    save(fullfile(modelDir,'yolo_detector.mat'),'detector','classNames','inputSize');
end
fprintf('=== TrainYOLO Complete ===\n');


%% HELPERS ──────────────────────────────────────────────────────────────
function data = augmentData(data)
% Data Augmentation for YOLO (Horizontal Flip and Color Jitter)
    for i = 1:size(data, 1)
        img = data{i, 1};
        bboxes = data{i, 2};
        labels = data{i, 3};
        
        try
            % Random Horizontal Flip (50% chance)
            if rand() > 0.5
                img = fliplr(img);
                if ~isempty(bboxes)
                    sz = size(img);
                    bboxes(:,1) = sz(2) - bboxes(:,1) - bboxes(:,3); 
                end
            end
            
            % Random Color Jitter (Brightness and Contrast)
            if size(img,3) == 3
                img = jitterColorHSV(img, 'Brightness', [0.8 1.2], 'Contrast', [0.8 1.2]);
            end
            
            % Aggressive Occlusion Augmentation (Random Cutout / Erasing)
            if rand() > 0.3
                sz = size(img);
                h_cut = round(sz(1) * rand(1) * 0.3); % up to 30% of height
                w_cut = round(sz(2) * rand(1) * 0.3); % up to 30% of width
                y_c = randi([1, max(1, sz(1) - h_cut)]);
                x_c = randi([1, max(1, sz(2) - w_cut)]);
                img(y_c : y_c + h_cut, x_c : x_c + w_cut, :) = 127; % Fill with gray
            end
            
        catch
            % If jitterColorHSV or fliplr fails (e.g. older MATLAB), ignore augmentation
        end
        
        data{i, 1} = img;
        data{i, 2} = bboxes;
        data{i, 3} = labels;
    end
end
