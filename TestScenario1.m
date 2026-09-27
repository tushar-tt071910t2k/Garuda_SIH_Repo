% TestScenario1.m - Run Scenario 1 standalone to verify correctness
disp('=== Testing Scenario 1: Village Road ===');
addpath(genpath('scenarios'));
addpath(genpath('utils'));
try
    metrics = Scenario1_VillageRoad();
    fprintf('\n--- RESULTS ---\n');
    fprintf('Goal Reached : %d\n', metrics.GoalReached);
    fprintf('Collision    : %d\n', metrics.CollisionOccurred);
    fprintf('Avg Speed    : %.2f m/s\n', mean(metrics.Speeds));
    fprintf('Braking Evts : %d\n', metrics.BrakingCount);
    fprintf('Min Clearance: %.2f m\n', min(metrics.MinDistances(isfinite(metrics.MinDistances))));
    disp('=== Scenario 1 PASSED ===');
catch e
    fprintf('=== Scenario 1 FAILED ===\n');
    fprintf('Error: %s\n', e.message);
    for k = 1:min(3, numel(e.stack))
        fprintf('  File: %s  Line: %d\n', e.stack(k).file, e.stack(k).line);
    end
end
exit;
