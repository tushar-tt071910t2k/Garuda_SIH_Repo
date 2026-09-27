% ProbeChasePlot.m - Find valid chasePlot properties in R2026a
disp('=== Probing chasePlot API ===');
scenario = drivingScenario('SampleTime',0.1,'StopTime',5);
road(scenario,[0 0 0; 50 0 0]);
ego = vehicle(scenario,'ClassID',1);
smoothTrajectory(ego,[0 0 0; 48 0 0],5);

fig = figure('Visible','off');
ax = axes(fig);

% Try each property one at a time
props = {'Meshes','ViewHeight','ViewDist','ViewPitch','FollowDistance','ZoomFactor'};
for i = 1:numel(props)
    try
        cp = chasePlot(ego,'Parent',ax,props{i},1);
        fprintf('  VALID:   %s\n', props{i});
    catch e
        fprintf('  INVALID: %s  -> %s\n', props{i}, e.message(1:min(80,end)));
    end
    clf(fig);
    ax = axes(fig);
end

% Try basic call
try
    cp = chasePlot(ego,'Parent',ax);
    fprintf('  Basic chasePlot: OK\n');
    p = properties(cp);
    fprintf('  Valid properties:\n');
    for i=1:numel(p); fprintf('    %s\n',p{i}); end
catch e
    fprintf('  Basic chasePlot failed: %s\n', e.message);
end

close(fig);
disp('=== Done ===');
exit;
