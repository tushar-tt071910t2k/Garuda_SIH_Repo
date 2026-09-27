% SyntaxCheck.m - Validate all scenario and utility files before full run
disp('=== SIH Project Syntax Check ===');
addpath(genpath('scenarios'));
addpath(genpath('utils'));

files = {
    'utils/CollisionAvoidance.m'
    'utils/MetricsLogger.m'
    'utils/LogStep.m'
    'utils/PrintSummary.m'
    'utils/PlotResults.m'
    'scenarios/Scenario1_VillageRoad.m'
    'scenarios/Scenario2_UrbanIntersection.m'
    'scenarios/Scenario3_HighwayMerge.m'
    'scenarios/Scenario4_MarketArea.m'
    'scenarios/Scenario5_CattleCrossing.m'
    'RunAllScenarios.m'
};

allOK = true;
for i = 1:numel(files)
    fname = files{i};
    [~, baseName, ~] = fileparts(fname);
    try
        % Use checkcode (MLint) for static analysis
        issues = checkcode(fname, '-cyc', '-id');
        if isempty(issues)
            fprintf('  ✅  %s — OK\n', baseName);
        else
            fprintf('  ⚠️  %s — %d lint warning(s):\n', baseName, numel(issues));
            for j = 1:min(3, numel(issues))
                fprintf('       Line %d: %s\n', issues(j).line, issues(j).message);
            end
        end
    catch e
        fprintf('  ❌  %s — ERROR: %s\n', baseName, e.message);
        allOK = false;
    end
end

fprintf('\n');
if allOK
    disp('=== All files passed syntax check! ===');
else
    disp('=== Some files have errors — check above ===');
end
exit;
