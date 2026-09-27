% CheckMoveAPI.m - Check what vehicle motion API is available in R2026a
disp('=== Checking Vehicle Motion API ===');
addpath(genpath('scenarios'));
addpath(genpath('utils'));

scenario = drivingScenario('SampleTime', 0.1);
road(scenario, [0 0 0; 100 0 0]);
ego = vehicle(scenario, 'ClassID', 1);
trajectory(ego, [0 0 0; 100 0 0], 5);

% Check what methods are available on actor
m = methods(ego);
fprintf('Actor methods:\n');
for i = 1:numel(m)
    fprintf('  %s\n', m{i});
end

% Also check if 'move' exists
hasmove = any(strcmp(m, 'move'));
fprintf('\nHas "move" method: %d\n', hasmove);
disp('=== Done ===');
exit;
