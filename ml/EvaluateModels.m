% EvaluateModels.m — Evaluate all 3 trained ML models
% Tests YOLO mAP, DeepLab IoU, CNN speed/steer error
clc;
addpath(genpath('../utils'));
addpath(genpath('.'));

fprintf('=== Evaluating All ML Models ===\n\n');

modelDir = fullfile(fileparts(mfilename('fullpath')), 'models');
dataDir  = fullfile(fileparts(mfilename('fullpath')), 'data');
results  = struct();

%% --- 1. Evaluate YOLO ---
fprintf('─── YOLO Object Detector ───\n');
yoloPath = fullfile(modelDir,'yolo_detector.mat');
if exist(yoloPath,'file')
    d = load(yoloPath);
    if ~isempty(d.detector)
        % Find test images
        testJSONs = dir(fullfile(dataDir,'test','**','*.json'));
        if ~isempty(testJSONs)
            fprintf('Evaluating on %d test JSON files...\n', numel(testJSONs));
            allData = cell(numel(testJSONs),1);
            for i=1:numel(testJSONs)
                allData{i} = ParseJSONAnnotations(fullfile(testJSONs(i).folder,testJSONs(i).name));
            end
            testData = vertcat(allData{:});

            % Compute mAP on first 20 images (fast eval)
            N = min(20, height(testData));
            detBboxes  = cell(N,1);
            detScores  = cell(N,1);
            detLabels  = cell(N,1);
            gtBboxes   = cell(N,1);
            gtLabels   = cell(N,1);

            for i=1:N
                try
                    img = imread(testData.ImageFile{i});
                    img = imresize(img, d.inputSize(1:2));
                    [b,s,l] = detect(d.detector, img, 'Threshold', 0.3);
                    detBboxes{i}=b; detScores{i}=s; detLabels{i}=l;
                catch
                    detBboxes{i}=zeros(0,4); detScores{i}=[]; detLabels{i}=categorical({});
                end
                gtBboxes{i} = testData.BBoxes{i};
                gtLabels{i} = testData.Labels{i};
            end

            try
                [ap, recall, prec] = evaluateObjectDetection( ...
                    detBboxes, detScores, detLabels, gtBboxes, gtLabels, 0.5);
                mAP = mean(ap);
                fprintf('  mAP@0.5: %.3f\n', mAP);
                fprintf('  Mean Precision: %.3f\n', mean(cellfun(@mean,num2cell(prec))));
                fprintf('  Mean Recall:    %.3f\n', mean(cellfun(@mean,num2cell(recall))));
                results.yolo_mAP = mAP;
            catch e
                fprintf('  mAP computation skipped: %s\n', e.message);
                results.yolo_mAP = NaN;
            end
        else
            fprintf('  No test data found — skipping mAP.\n');
            results.yolo_mAP = NaN;
        end

        % Speed test
        dummyFrame = uint8(rand(d.inputSize)*255);
        t0 = tic;
        for i=1:5; detect(d.detector, dummyFrame, 'Threshold',0.3); end
        fps = 5/toc(t0);
        fprintf('  Inference speed: %.1f FPS\n', fps);
        results.yolo_fps = fps;
    else
        fprintf('  Model is empty (training failed).\n');
        results.yolo_mAP = NaN;
    end
else
    fprintf('  [NOT FOUND] Run TrainYOLO.m first.\n');
    results.yolo_mAP = NaN;
end

%% --- 2. Evaluate DeepLabv3+ ---
fprintf('\n─── DeepLabv3+ Segmentation ───\n');
dlPath = fullfile(modelDir,'deeplab_net.mat');
if exist(dlPath,'file')
    d2 = load(dlPath);
    if ~isempty(d2.net)
        % Find IDD label pairs
        imgFiles = dir(fullfile(dataDir,'idd','**','*.jpg'));
        lblFiles = dir(fullfile(dataDir,'idd','**','*.png'));

        if ~isempty(imgFiles) && ~isempty(lblFiles)
            N = min(10, numel(imgFiles));
            iouVals = zeros(N,1);
            for i=1:N
                try
                    img  = imresize(imread(imgFiles(i).name), d2.INPUT_SIZE(1:2));
                    pred = semanticseg(img, d2.net);
                    lbl  = imresize(imread(lblFiles(i).name), d2.INPUT_SIZE(1:2),'nearest');
                    gtRoad  = lbl(:,:,1) < 50 & lbl(:,:,2) > 80 & lbl(:,:,3) > 150;
                    predRoad = pred == "road";
                    inter = sum(gtRoad(:) & predRoad(:));
                    union = sum(gtRoad(:) | predRoad(:));
                    iouVals(i) = inter / max(union,1);
                catch
                    iouVals(i) = NaN;
                end
            end
            mIoU = mean(iouVals(~isnan(iouVals)));
            fprintf('  Mean IoU (road class): %.3f\n', mIoU);
            results.deeplab_mIoU = mIoU;
        else
            fprintf('  No IDD images found — running synthetic test...\n');
            dummyImg = uint8(rand(d2.INPUT_SIZE)*255);
            predMask = semanticseg(dummyImg, d2.net);
            roadFrac = mean(predMask=="road",'all');
            fprintf('  Synthetic road fraction: %.1f%%\n', roadFrac*100);
            results.deeplab_mIoU = NaN;
        end
    else
        fprintf('  Model is empty.\n');
        results.deeplab_mIoU = NaN;
    end
else
    fprintf('  [NOT FOUND] Run TrainSegmentation.m first.\n');
    results.deeplab_mIoU = NaN;
end

%% --- 3. Evaluate CNN Behavioral Clone ---
fprintf('\n─── CNN Behavioral Clone ───\n');
cnnPath = fullfile(modelDir,'cnn_control.mat');
if exist(cnnPath,'file')
    d3 = load(cnnPath);
    if ~isempty(d3.net)
        % Generate test frames from a quick simulation
        fprintf('  Generating test frames...\n');
        scenario = drivingScenario('SampleTime',0.1,'StopTime',5);
        road(scenario,[0 0 0;50 0 0]);
        ego = vehicle(scenario,'ClassID',1);
        smoothTrajectory(ego,[0 0 0;48 0 0],8);

        steerErrors=[]; speedErrors=[];
        while advance(scenario)
            st  = ego.state;
            vel = st.Velocity;
            pos = st.Position;
            gtSpd   = norm(vel(1:2));
            gtSteer = atan2(vel(2), vel(1)+1e-9)*0.3;

            frame = SyntheticCamera(scenario, pos, d3.FRAME_SIZE(1:2));
            [predSteer, predSpd, ~] = RunCNNControl(frame, d3.net, d3.FRAME_SIZE, 8);

            steerErrors(end+1) = abs(predSteer - gtSteer);
            speedErrors(end+1) = abs(predSpd   - gtSpd);
        end

        fprintf('  Mean Steer Error: %.4f rad\n', mean(steerErrors));
        fprintf('  Mean Speed Error: %.3f m/s\n',  mean(speedErrors));
        results.cnn_steerMAE = mean(steerErrors);
        results.cnn_speedMAE = mean(speedErrors);
    else
        fprintf('  Model is empty.\n');
        results.cnn_steerMAE = NaN; results.cnn_speedMAE = NaN;
    end
else
    fprintf('  [NOT FOUND] Run TrainBehavioralClone.m first.\n');
    results.cnn_steerMAE = NaN; results.cnn_speedMAE = NaN;
end

%% --- Summary ---
fprintf('\n╔══════════════════════════════════════════╗\n');
fprintf('║           MODEL EVALUATION SUMMARY       ║\n');
fprintf('╠══════════════════════════════════════════╣\n');
fprintf('║  YOLO mAP@0.5      : %-8s             ║\n', fmtVal(results.yolo_mAP));
fprintf('║  YOLO FPS          : %-8s             ║\n', fmtVal(results.yolo_fps));
fprintf('║  DeepLab mIoU      : %-8s             ║\n', fmtVal(results.deeplab_mIoU));
fprintf('║  CNN Steer MAE     : %-8s rad          ║\n', fmtVal(results.cnn_steerMAE));
fprintf('║  CNN Speed MAE     : %-8s m/s          ║\n', fmtVal(results.cnn_speedMAE));
fprintf('╚══════════════════════════════════════════╝\n');

save(fullfile(modelDir,'eval_results.mat'), 'results');
fprintf('\nResults saved: ml/models/eval_results.mat\n');
fprintf('=== Evaluation Complete ===\n');

function s = fmtVal(v)
    if isnan(v)||isempty(v); s='N/A';
    else; s = sprintf('%.4f',v); end
end
