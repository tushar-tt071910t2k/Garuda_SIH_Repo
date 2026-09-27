% =========================================================================
%  RunAllScenarios.m — SIH Master Runner (ML-Enhanced)
%  Runs all 5 Indian road scenarios with ML integration
%  Generates performance dashboard
% =========================================================================
%  HOW TO RUN:
%    cd('e:\sih_matlab')
%    RunAllScenarios
%
%  SLOW-MOTION: Each scenario runs at 10x slower than real-time
%    so you can clearly see the ego vehicle navigate around obstacles.
%    All obstacles (pedestrians, cows, bikes, trucks) are animated.
% =========================================================================

clc; clear; close all;

%% ── Fix paths: works from ANY directory ──────────────────────────────────
ROOT = fileparts(mfilename('fullpath'));   % always e:\sih_matlab
addpath(ROOT);                             % add root itself first
addpath(genpath(fullfile(ROOT,'scenarios')));
addpath(genpath(fullfile(ROOT,'utils')));
addpath(genpath(fullfile(ROOT,'ml')));

fprintf('╔══════════════════════════════════════════════════════════╗\n');
fprintf('║   SIH 2025 — Autonomous Vehicle: Indian Road Scenarios   ║\n');
fprintf('║   Adaptive Path Planning + ML (YOLO / DeepLab / CNN)    ║\n');
fprintf('╚══════════════════════════════════════════════════════════╝\n\n');

%% ---- Model Status Report ----
modelDir = fullfile(ROOT,'ml','models');
yoloPath = fullfile(modelDir,'yolo_detector.mat');
dlPath   = fullfile(modelDir,'deeplab_net.mat');
cnnPath  = fullfile(modelDir,'cnn_control.mat');

fprintf('╔══════════════════════════════════════════════════════════╗\n');
fprintf('║              ML MODEL STATUS                             ║\n');
fprintf('╠══════════════════════════════════════════════════════════╣\n');
if exist(yoloPath,'file')
    d=load(yoloPath,'detector');
    if ~isempty(d.detector) && isstruct(d.detector)
        fprintf('║  ✅ YOLO/CNN Detector : TRAINED (51.6%% acc, 14413 IDD patches) ║\n');
    else
        fprintf('║  ⚠️  YOLO Detector    : File exists but empty — retrain        ║\n');
    end
else
    fprintf('║  ❌ YOLO Detector    : NOT TRAINED — run ml/TrainCustomDetector  ║\n');
end
if exist(dlPath,'file')
    fprintf('║  ✅ DeepLab Segment  : TRAINED                               ║\n');
else
    fprintf('║  ❌ DeepLab Segment  : NOT TRAINED (using geometric fallback)  ║\n');
    fprintf('║     → Run: cd ml; TrainSegmentation                          ║\n');
end
if exist(cnnPath,'file')
    fprintf('║  ✅ CNN Behavioural  : TRAINED                               ║\n');
else
    fprintf('║  ❌ CNN Behavioural  : NOT TRAINED (using rule-based fallback) ║\n');
    fprintf('║     → Run: cd ml; TrainBehavioralClone                       ║\n');
end
fprintf('╠══════════════════════════════════════════════════════════╣\n');
fprintf('║  Slow-motion: 10x slower so all obstacle motion is visible  ║\n');
fprintf('╚══════════════════════════════════════════════════════════╝\n\n');

%% ---- Load ML Models ----
fprintf('Loading ML models...\n');
mlModels = LoadMLModels();
fprintf('  YOLO    : %s\n', statusStr(~isempty(mlModels.yolo)));
fprintf('  DeepLab : %s\n', statusStr(~isempty(mlModels.deeplab)));
fprintf('  CNN     : %s\n\n', statusStr(~isempty(mlModels.cnn)));

%% ---- Run All 5 Scenarios ----
allMetrics = cell(1,5);
scenarioFns = {
    @() Scenario1_VillageRoad(mlModels),      'SCENARIO 1 of 5: Unmarked Village Road'
    @() Scenario2_UrbanIntersection(mlModels),'SCENARIO 2 of 5: Busy Urban Intersection'
    @() Scenario3_HighwayMerge(mlModels),     'SCENARIO 3 of 5: Highway Merge'
    @() Scenario4_MarketArea(mlModels),       'SCENARIO 4 of 5: Dense Market Area'
    @() Scenario5_CattleCrossing(mlModels),   'SCENARIO 5 of 5: Cattle Crossing'
};

for i = 1:5
    fprintf('\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
    fprintf('  %s\n', scenarioFns{i,2});
    fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
    fprintf('  Press any key in figures to skip, or wait for completion\n\n');
    close all;  % close previous scenario figures quickly
    drawnow;
    try
        allMetrics{i} = scenarioFns{i,1}();
        m = allMetrics{i};
        if m.GoalReached && ~m.CollisionOccurred
            fprintf('  [S%d] ✅ PASSED  — Goal reached, no collision\n', i);
        elseif m.GoalReached
            fprintf('  [S%d] ⚠️  GOAL but collision detected\n', i);
        else
            fprintf('  [S%d] ❌ Did not reach goal\n', i);
        end
    catch e
        fprintf('  [S%d] ❌ Error: %s\n', i, e.message);
        allMetrics{i} = createDummyMetrics(sprintf('Scenario%d',i));
    end

    if i < 5
        fprintf('\n  ⏳ Next scenario starts in 2 seconds...\n');
        pause(2);  % Short pause between scenarios (was implicit longer)
    end
end

%% ---- Dashboard ----
fprintf('\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('  Generating ML-Enhanced Performance Dashboard...\n');
PlotResults(allMetrics);

%% ---- Final Report ----
fprintf('\n╔══════════════════════════════════════════════════════════╗\n');
fprintf('║                   FINAL SCENARIO REPORT                 ║\n');
fprintf('╠══════════════════════════════════════════════════════════╣\n');
fprintf('║  %-20s  %-8s  %-8s  %-8s ║\n','Scenario','Goal','Collide','AvgSpd');
fprintf('╠══════════════════════════════════════════════════════════╣\n');
for i = 1:5
    m   = allMetrics{i};
    nm  = strrep(m.ScenarioName,'_',' ');
    g   = boolStr(m.GoalReached,     'YES ✅','NO  ❌');
    c   = boolStr(m.CollisionOccurred,'YES ⚠️ ','NO  ✅');
    spd = mean(m.Speeds);
    fprintf('║  %-20s  %-8s  %-8s  %5.2f m/s ║\n', nm(1:min(20,end)), g, c, spd);
end
fprintf('╚══════════════════════════════════════════════════════════╝\n');
passCount = sum(cellfun(@(m) m.GoalReached && ~m.CollisionOccurred, allMetrics));
fprintf('\n  Scenarios Passed: %d / 5\n', passCount);
fprintf('  Completion Rate : %.0f%%\n\n', passCount/5*100);


%% ---- Helpers ----
function models = LoadMLModels()
    ROOT     = fileparts(mfilename('fullpath'));
    modelDir = fullfile(ROOT, 'ml', 'models');
    models.yolo = []; models.deeplab = []; models.cnn = [];

    % YOLO / CNN Detector
    yoloPath = fullfile(modelDir,'yolo_detector.mat');
    if exist(yoloPath,'file')
        try; d=load(yoloPath,'detector'); models.yolo=d.detector;
             fprintf('  Loaded YOLO/CNN Detector from %s\n', yoloPath); catch; end
    else
        fprintf('  [INFO] YOLO not found — colour-heuristic fallback\n');
    end

    % DeepLab Segmentation (supports both full DeepLab and fast segNet)
    dlPath = fullfile(modelDir,'deeplab_net.mat');
    if exist(dlPath,'file')
        try
            d = load(dlPath);
            if isfield(d,'segNet');  models.deeplab = d.segNet;
            elseif isfield(d,'net'); models.deeplab = d.net; end
            fprintf('  Loaded DeepLab from %s\n', dlPath);
        catch; end
    else
        fprintf('  [INFO] DeepLab not found — geometric fallback\n');
    end

    % CNN Behavioural Clone (supports both 'net' and 'cnnNet' variable names)
    cnnPath = fullfile(modelDir,'cnn_control.mat');
    if exist(cnnPath,'file')
        try
            d = load(cnnPath);
            if isfield(d,'cnnNet'); models.cnn = d.cnnNet;
            elseif isfield(d,'net'); models.cnn = d.net; end
            fprintf('  Loaded CNN Behavioural Clone from %s\n', cnnPath);
        catch; end
    else
        fprintf('  [INFO] CNN not found — rule-based fallback\n');
    end
end
function m = createDummyMetrics(name)
    m=MetricsLogger(name); m.Speeds=[0]; m.SteerAngles=[0];
    m.MinDistances=[Inf]; m.GoalReached=false; m.CollisionOccurred=false;
end
function s = statusStr(ok)
    if ok; s='Loaded ✅ (active)'; else; s='Fallback mode'; end
end
function r = boolStr(cond, a, b)
    if cond; r=a; else; r=b; end
end
