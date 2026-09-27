% ProbeEgoControl.m - Find the correct way to control ego in R2026a
disp('=== Probing Ego Control API ===');

scenario = drivingScenario('SampleTime', 0.05, 'StopTime', 15);
road(scenario, [0 0 0; 100 0 0]);
ego = vehicle(scenario, 'ClassID', 1, 'Position', [2 0 0]);

% Method 1: smoothTrajectory - adaptive speed along fixed waypoints
waypts = [2 0 0; 50 0 0; 98 0 0];
speeds = [8 8 8];  % m/s at each waypoint
smoothTrajectory(ego, waypts, speeds);

disp('smoothTrajectory accepted OK');

% Check "state" method
s = ego.state;
fprintf('ego.state class: %s, size: %s\n', class(s), mat2str(size(s)));

% Show state
disp('Initial ego state:');
disp(s);

% Check "path" method  
p = ego.path;
fprintf('ego.path class: %s\n', class(p));

disp('=== Done ===');
exit;
