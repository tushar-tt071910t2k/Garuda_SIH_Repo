% Test Suite for SafePathPlanner MATLAB Port

disp('==========================================================');
disp('RUNNING RIGOROUS SYNTHETIC TEST SUITE (MATLAB)');
disp('==========================================================');

planner = SafePathPlanner();
grid_shape = [planner.grid_h, planner.grid_w];
valid_grid = ones(grid_shape, 'uint8');

%% 4. Empty Road
disp('Running Test: Empty Road...');
res = planner.planFromBEV(valid_grid, {}, {});
if strcmp(res.status, 'PLANNER_SAFE') && res.verified_safe
    disp('[PASS] Test: Empty Road.');
else
    error('[FAIL] Test: Empty Road.');
end

%% 5. Continuous Blocked Road
disp('Running Test: Completely Blocked Road...');
x_vals = linspace(-0.5, 1.5, 20);
wall = cell(length(x_vals), 1);
for i = 1:length(x_vals)
    x = x_vals(i);
    wall{i} = [x, 2.5; x+0.1, 2.5; x+0.1, 2.6; x, 2.6];
end
res = planner.planFromBEV(valid_grid, {}, wall);
if strcmp(res.status, 'NO_SAFE_ROUTE')
    disp('[PASS] Test: Completely Blocked Road -> NO_SAFE_ROUTE.');
else
    error('[FAIL] Test: Completely Blocked Road (Status: %s).', res.status);
end

%% 7. Obstacle Perturbation Robustness
disp('Running Test: Obstacle Perturbation...');
success_count = 0;
shifts = [-0.05, 0.0, 0.05];
for i = 1:length(shifts)
    shift = shifts(i);
    poly = {[0.5+shift, 2.0; 0.7+shift, 2.0; 0.7+shift, 2.2; 0.5+shift, 2.2]};
    ego_state = struct('x', 0.0, 'z', 0.8);
    r = planner.planFromBEV(valid_grid, {}, poly, ego_state);
    if strcmp(r.status, 'PLANNER_SAFE')
        success_count = success_count + 1;
    end
end

if success_count == 3
    disp('[PASS] Test: Obstacle Perturbation Handled Safely.');
else
    error('[FAIL] Perturbation robustness failed (Success count: %d).', success_count);
end

disp('==========================================================');
disp('ALL SYNTHETIC TESTS PASSED IN MATLAB.');
disp('==========================================================');
