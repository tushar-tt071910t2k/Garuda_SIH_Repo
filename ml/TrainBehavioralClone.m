% TrainBehavioralClone.m
% =========================================================================
% Train CNN Behavioral Cloning model
% Records synthetic camera frames from simulation + ego speed/steer labels
% CNN: [120x160x3 frame] → [steering_angle, speed_command]
% =========================================================================
% OUTPUT: ml/models/cnn_control.mat
% =========================================================================

clc; clear;
addpath(genpath('../scenarios'));
addpath(genpath('../utils'));
addpath(genpath('.'));

fprintf('=== CNN Behavioral Cloning Training ===\n\n');

FRAME_SIZE   = [120 160 3];
RECORD_DIR   = fullfile(fileparts(mfilename('fullpath')), 'recordings');
MODEL_DIR    = fullfile(fileparts(mfilename('fullpath')), 'models');

%% --- 1. Record Training Data from Scenarios ---
fprintf('Recording driving frames from scenarios...\n');

allFrames  = [];
allLabels  = [];

% Record from Scenario 1 (village road)
fprintf('  Recording Scenario 1...\n');
[f1, l1] = RecordScenario('village', FRAME_SIZE);
allFrames  = cat(4, allFrames, f1);
allLabels  = [allLabels; l1];

% Record from Scenario 2 (intersection)
fprintf('  Recording Scenario 2...\n');
[f2, l2] = RecordScenario('intersection', FRAME_SIZE);
allFrames  = cat(4, allFrames, f2);
allLabels  = [allLabels; l2];

% Record from Scenario 3 (highway)
fprintf('  Recording Scenario 3...\n');
[f3, l3] = RecordScenario('highway', FRAME_SIZE);
allFrames  = cat(4, allFrames, f3);
allLabels  = [allLabels; l3];

fprintf('Total training frames collected: %d\n', size(allFrames,4));

%% --- 2. Shuffle and Split ---
N   = size(allFrames, 4);
idx = randperm(N);
splitPt = round(0.85 * N);

trainIdx = idx(1:splitPt);
valIdx   = idx(splitPt+1:end);

trainFrames = allFrames(:,:,:,trainIdx);
trainLabels = allLabels(trainIdx,:);
valFrames   = allFrames(:,:,:,valIdx);
valLabels   = allLabels(valIdx,:);

%% --- 3. Build CNN (NVIDIA PilotNet style, adapted for MATLAB) ---
fprintf('\nBuilding PilotNet-style CNN...\n');

layers = [
    imageInputLayer(FRAME_SIZE, 'Normalization', 'zerocenter', 'Name', 'input')

    % Convolutional feature extraction
    convolution2dLayer([5 5], 24, 'Stride', [2 2], 'Padding', 'same', 'Name', 'conv1')
    batchNormalizationLayer('Name','bn1')
    eluLayer('Name','elu1')

    convolution2dLayer([5 5], 36, 'Stride', [2 2], 'Padding', 'same', 'Name', 'conv2')
    batchNormalizationLayer('Name','bn2')
    eluLayer('Name','elu2')

    convolution2dLayer([5 5], 48, 'Stride', [2 2], 'Padding', 'same', 'Name', 'conv3')
    batchNormalizationLayer('Name','bn3')
    eluLayer('Name','elu3')

    convolution2dLayer([3 3], 64, 'Padding', 'same', 'Name', 'conv4')
    batchNormalizationLayer('Name','bn4')
    eluLayer('Name','elu4')

    convolution2dLayer([3 3], 64, 'Padding', 'same', 'Name', 'conv5')
    batchNormalizationLayer('Name','bn5')
    eluLayer('Name','elu5')

    % Fully connected regression head
    flattenLayer('Name','flat')
    fullyConnectedLayer(1164, 'Name','fc1')
    dropoutLayer(0.3,  'Name','drop1')
    eluLayer('Name','elu6')

    fullyConnectedLayer(100, 'Name','fc2')
    eluLayer('Name','elu7')

    fullyConnectedLayer(50, 'Name','fc3')
    eluLayer('Name','elu8')

    fullyConnectedLayer(10, 'Name','fc4')
    eluLayer('Name','elu9')

    fullyConnectedLayer(2, 'Name','output')   % [steering_angle, speed]
    regressionLayer('Name','regression')
];

%% --- 4. Training Options ---
options = trainingOptions('adam', ...
    'MaxEpochs',            40, ...
    'MiniBatchSize',        32, ...
    'InitialLearnRate',     1e-3, ...
    'LearnRateSchedule',    'piecewise', ...
    'LearnRateDropFactor',  0.5, ...
    'LearnRateDropPeriod',  15, ...
    'Shuffle',              'every-epoch', ...
    'ValidationData',       {valFrames, valLabels}, ...
    'ValidationFrequency',  30, ...
    'VerboseFrequency',     20, ...
    'Plots',                'training-progress', ...
    'CheckpointPath',        MODEL_DIR);

%% --- 5. Train ---
fprintf('\nTraining CNN Behavioral Clone...\n');
net = trainNetwork(trainFrames, trainLabels, layers, options);

%% --- 6. Save ---
modelPath = fullfile(MODEL_DIR, 'cnn_control.mat');
save(modelPath, 'net', 'FRAME_SIZE');
fprintf('\n[SUCCESS] CNN Behavioral Clone saved: %s\n', modelPath);

% Sample prediction
sampleFrame = trainFrames(:,:,:,1);
pred = predict(net, sampleFrame);
fprintf('  Sample prediction: steer=%.3f rad, speed=%.2f m/s\n', pred(1), pred(2));

fprintf('=== TrainBehavioralClone Complete ===\n');


%% =========== Helper: Record frames from a simulated scenario =========== %%
function [frames, labels] = RecordScenario(scenarioType, frameSize)
% Runs a quick simulation and captures synthetic frames + control labels

    scenario = drivingScenario('SampleTime', 0.1, 'StopTime', 20);

    switch scenarioType
        case 'village'
            road(scenario,[0 0 0;50 0 0;100 5 0;150 5 0],'Lanes',lanespec(1,'Width',3.5));
            ego = vehicle(scenario,'ClassID',1);
            smoothTrajectory(ego,[0 0 0;50 0 0;100 5 0;148 5 0],6*ones(4,1));
            nomSpeed = 6;

        case 'intersection'
            road(scenario,[-60 0 0;60 0 0],'Lanes',lanespec(2));
            road(scenario,[0 -60 0;0 60 0],'Lanes',lanespec(2));
            ego = vehicle(scenario,'ClassID',1);
            smoothTrajectory(ego,[-58 -1.75 0;58 -1.75 0],7*ones(2,1));
            nomSpeed = 7;

        case 'highway'
            road(scenario,[0 0 0;300 0 0],'Lanes',lanespec(3));
            ego = vehicle(scenario,'ClassID',1);
            smoothTrajectory(ego,[0 -1.75 0;150 1.75 0;298 1.75 0],18*ones(3,1));
            nomSpeed = 18;

        otherwise
            road(scenario,[0 0 0;100 0 0]);
            ego = vehicle(scenario,'ClassID',1);
            smoothTrajectory(ego,[0 0 0;98 0 0],6*ones(2,1));
            nomSpeed = 6;
    end

    frames = zeros([frameSize 0], 'uint8');
    labels = zeros(0, 2);

    fig = figure('Visible','off','Position',[0 0 frameSize(2)*2 frameSize(1)*2]);
    ax  = axes(fig,'Color',[0.1 0.1 0.15]);
    chasePlot(ego,'Parent',ax);

    while advance(scenario)
        st      = ego.state;
        vel     = st.Velocity;
        spd     = norm(vel(1:2));
        heading = atan2(vel(2), vel(1)+1e-9);

        % Steering approximation: rate of heading change
        steer = heading * 0.3;  % scaled proxy
        spd_n = spd / nomSpeed; % normalized 0-1

        % Capture synthetic frame
        scenario.updatePlots();
        drawnow;
        fdata = getframe(ax);
        frame = imresize(fdata.cdata, frameSize(1:2));

        frames = cat(4, frames, frame);
        labels = [labels; steer, spd_n * nomSpeed];

        if isDone(scenario); break; end
    end

    close(fig);
    fprintf('    Captured %d frames from %s scenario\n', size(frames,4), scenarioType);
end
