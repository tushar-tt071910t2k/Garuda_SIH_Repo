% =========================================================================
%  JUDGE_DEMO.m  — SIH 2025 Judge Presentation Runner
%  Run this file in MATLAB to present all 5 scenarios
%
%  HOW TO RUN:
%    1. Open MATLAB
%    2. Type:  cd('e:\sih_matlab')
%    3. Type:  JUDGE_DEMO
%    4. Two windows will open per scenario — Chase View + Top View
%    5. Press Enter in Command Window to advance to next scenario
% =========================================================================

clc; clear; close all;
fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════╗\n');
fprintf('║      SIH 2025 — Autonomous Navigation Demo              ║\n');
fprintf('║      Adaptive Path Planning for Indian Roads            ║\n');
fprintf('╠══════════════════════════════════════════════════════════╣\n');
fprintf('║  Team: [Your Team Name]                                 ║\n');
fprintf('║  ML: YOLO + DeepLab + CNN Behavioral Clone              ║\n');
fprintf('║  Speed: 10× slow-motion so judges can see clearly       ║\n');
fprintf('╚══════════════════════════════════════════════════════════╝\n\n');

%% Fix paths
ROOT = fileparts(mfilename('fullpath'));
addpath(ROOT);
addpath(genpath(fullfile(ROOT,'scenarios')));
addpath(genpath(fullfile(ROOT,'utils')));
addpath(genpath(fullfile(ROOT,'ml')));

%% Load ML Models
fprintf('Loading ML models...\n');
mlModels = LoadMLModelsLocal(ROOT);
fprintf('  YOLO    : %s\n', ternStr(~isempty(mlModels.yolo),   'Trained ✅', 'Fallback mode'));
fprintf('  DeepLab : %s\n', ternStr(~isempty(mlModels.deeplab),'Trained ✅', 'Fallback mode'));
fprintf('  CNN     : %s\n', ternStr(~isempty(mlModels.cnn),    'Trained ✅', 'Fallback mode'));
fprintf('\n');

scenarioList = {
    @() Scenario1_VillageRoad(mlModels),      'SCENARIO 1 / 5 — Unmarked Village Road'
    @() Scenario2_UrbanIntersection(mlModels),'SCENARIO 2 / 5 — Busy Urban Intersection'
    @() Scenario3_HighwayMerge(mlModels),     'SCENARIO 3 / 5 — Highway Merge & Overtake'
    @() Scenario4_MarketArea(mlModels),       'SCENARIO 4 / 5 — Dense Market Area'
    @() Scenario5_CattleCrossing(mlModels),   'SCENARIO 5 / 5 — Cattle Crossing'
};

allMetrics = cell(1,5);

for i = 1:5
    fprintf('\n');
    fprintf('┌──────────────────────────────────────────────────────────┐\n');
    fprintf('│  %-56s│\n', scenarioList{i,2});
    fprintf('└──────────────────────────────────────────────────────────┘\n');
    
    fprintf('\n  What judges will see:\n');
    switch i
        case 1; fprintf('  • Ego (GREEN car) navigates a curved village road\n');
                fprintf('  • Pedestrians cross, bullock cart slow-moves ahead\n');
                fprintf('  • Oncoming bike, dog darting, child running\n');
        case 2; fprintf('  • Ego approaches a 4-way intersection\n');
                fprintf('  • Auto, bus, motorbike cross at right angles\n');
                fprintf('  • Ego yields correctly before crossing\n');
        case 3; fprintf('  • Ego merges from slip road onto 3-lane highway\n');
                fprintf('  • 2 slow trucks, 2 fast cars overtake or get overtaken\n');
                fprintf('  • Motorbike weaves, oncoming car in opposite lane\n');
        case 4; fprintf('  • Very slow navigation through narrow market street\n');
                fprintf('  • Pushcart, cow, pedestrians ×3, oncoming auto\n');
                fprintf('  • Ego stops for pedestrians, then continues\n');
        case 5; fprintf('  • Ego moves at speed, then encounters cattle herd\n');
                fprintf('  • 4 cows + shepherd crossing at x=100-130m\n');
                fprintf('  • Ego brakes, waits, then continues carefully\n');
    end
    
    fprintf('\n  >> Press ENTER to start Scenario %d (or Ctrl+C to quit)\n', i);
    input('', 's');
    
    close all; drawnow;
    
    try
        fprintf('\n  [Running...] Watch the two figure windows.\n\n');
        allMetrics{i} = scenarioList{i,1}();
        m = allMetrics{i};
        fprintf('\n  ── Result ──────────────────────────────────────\n');
        if m.GoalReached && ~m.CollisionOccurred
            fprintf('  ✅ PASSED  — Goal reached, zero collisions\n');
        elseif m.GoalReached
            fprintf('  ⚠️  GOAL reached — but collision detected\n');
        else
            fprintf('  ❌ Did not reach goal\n');
        end
        fprintf('  Avg speed : %.2f m/s (%.1f km/h)\n', mean(m.Speeds), mean(m.Speeds)*3.6);
        fprintf('  ────────────────────────────────────────────────\n');
    catch e
        fprintf('  ❌ Error in scenario %d: %s\n', i, e.message);
        allMetrics{i} = dummyMetrics(sprintf('Scenario%d',i));
    end
    
    if i < 5
        fprintf('\n  ⏳ Ready for next scenario. Press ENTER to continue...\n');
        input('', 's');
    end
end

%% Final Summary Table
fprintf('\n\n');
fprintf('╔══════════════════════════════════════════════════════════╗\n');
fprintf('║              FINAL DEMO RESULTS SUMMARY                 ║\n');
fprintf('╠══════════════════════════════════════════════════════════╣\n');
fprintf('║  %-22s  %-6s  %-8s  %-8s ║\n','Scenario','Goal','Collide','AvgSpd');
fprintf('╠══════════════════════════════════════════════════════════╣\n');
for i = 1:5
    m   = allMetrics{i};
    nm  = strrep(m.ScenarioName,'_',' ');
    g   = ternStr(m.GoalReached,      'YES ✅','NO  ❌');
    c   = ternStr(m.CollisionOccurred,'YES ⚠️ ','NO  ✅');
    spd = mean(m.Speeds);
    fprintf('║  %-22s  %-6s  %-8s  %4.1fm/s  ║\n', nm(1:min(22,end)), g, c, spd);
end
fprintf('╠══════════════════════════════════════════════════════════╣\n');
passCount = sum(cellfun(@(m) m.GoalReached && ~m.CollisionOccurred, allMetrics));
fprintf('║  Scenarios Passed : %d / 5  (%.0f%% success rate)           ║\n', passCount, passCount/5*100);
fprintf('╚══════════════════════════════════════════════════════════╝\n\n');

%% Plot results dashboard
PlotResults(allMetrics);
fprintf('✅ Demo Complete! Performance dashboard shown above.\n\n');


%% ── Helper functions ─────────────────────────────────────────────────────
function models = LoadMLModelsLocal(ROOT)
    modelDir = fullfile(ROOT, 'ml', 'models');
    models.yolo = []; models.deeplab = []; models.cnn = [];
    yoloPath = fullfile(modelDir,'yolo_detector.mat');
    if exist(yoloPath,'file')
        try; d=load(yoloPath,'detector'); models.yolo=d.detector; catch; end
    end
    dlPath = fullfile(modelDir,'deeplab_net.mat');
    if exist(dlPath,'file')
        try
            d = load(dlPath);
            if isfield(d,'segNet');  models.deeplab = d.segNet;
            elseif isfield(d,'net'); models.deeplab = d.net; end
        catch; end
    end
    cnnPath = fullfile(modelDir,'cnn_control.mat');
    if exist(cnnPath,'file')
        try
            d = load(cnnPath);
            if isfield(d,'cnnNet'); models.cnn = d.cnnNet;
            elseif isfield(d,'net'); models.cnn = d.net; end
        catch; end
    end
end

function s = ternStr(cond, a, b)
    if cond; s = a; else; s = b; end
end

function m = dummyMetrics(name)
    m = MetricsLogger(name);
    m.Speeds = [0]; m.SteerAngles = [0];
    m.MinDistances = [Inf]; m.GoalReached = false; m.CollisionOccurred = false;
end
