% ProbeViz.m - Find correct visualization API in R2026a
disp('=== Probing Visualization API ===');
scenario = drivingScenario('SampleTime',0.1,'StopTime',3);
road(scenario,[0 0 0; 50 0 0]);
ego = vehicle(scenario,'ClassID',1);
smoothTrajectory(ego,[0 0 0; 48 0 0],5);
ped = actor(scenario,'ClassID',4,'Length',0.5,'Width',0.5,'Height',1.7);
trajectory(ped,[10 -3 0; 10 3 0],1.5);

fig = figure('Visible','off','Position',[100 100 800 500]);
ax = axes(fig);

% Test 1: chasePlot with NO output
disp('Test 1: chasePlot(ego) no output...');
try
    chasePlot(ego);
    disp('  OK - no Parent');
catch e; fprintf('  FAIL: %s\n',e.message(1:min(100,end))); end

% Test 2: chasePlot with Parent, no output
clf(fig); ax=axes(fig);
disp('Test 2: chasePlot(ego, Parent, ax) no output...');
try
    chasePlot(ego, 'Parent', ax);
    disp('  OK');
catch e; fprintf('  FAIL: %s\n',e.message(1:min(100,end))); end

% Test 3: plot(scenario) - scenario-level plot
clf(fig); ax=axes(fig);
disp('Test 3: plot(scenario)...');
try
    plot(scenario);
    disp('  OK');
    disp('  plot(scenario) properties:');
catch e; fprintf('  FAIL: %s\n',e.message(1:min(100,end))); end

% Test 4: birdEyePlot
clf(fig); ax=axes(fig);
disp('Test 4: birdEyePlot...');
try
    bep = birdEyePlot('Parent',ax,'XLim',[-5 55],'YLim',[-10 10]);
    disp('  birdEyePlot OK');
    op = outlinePlotter(bep);
    disp('  outlinePlotter OK');
catch e; fprintf('  FAIL: %s\n',e.message(1:min(100,end))); end

% Test 5: plotSim3d
disp('Test 5: ego.plotSim3d...');
try
    m = methods(ego);
    hasSim3d = any(strcmp(m,'plotSim3d'));
    fprintf('  plotSim3d method exists: %d\n', hasSim3d);
catch e; fprintf('  FAIL: %s\n',e.message(1:min(100,end))); end

% Test 6: try scenario.plot
disp('Test 6: scenario methods...');
try
    m = methods(scenario);
    fprintf('  scenario methods: %s\n', strjoin(m(:)', ', '));
catch e; fprintf('  FAIL: %s\n',e.message(1:min(100,end))); end

close all;
disp('=== Done ===');
exit;
