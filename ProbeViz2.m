% ProbeViz2.m - Test birdEyePlot without Parent, and updatePlots
disp('=== Probing birdEyePlot + updatePlots ===');
scenario = drivingScenario('SampleTime',0.1,'StopTime',3);
road(scenario,[0 0 0; 50 0 0]);
ego = vehicle(scenario,'ClassID',1);
smoothTrajectory(ego,[0 0 0; 48 0 0],5);

% Test birdEyePlot with no Parent
disp('Test A: birdEyePlot no Parent...');
try
    bep = birdEyePlot();
    disp('  OK'); disp(class(bep));
catch e; fprintf('  FAIL: %s\n',e.message(1:min(120,end))); end

% Test birdEyePlot with XLim/YLim only
disp('Test B: birdEyePlot(XLim,YLim)...');
try
    bep = birdEyePlot('XLim',[-5 55],'YLim',[-10 10]);
    disp('  OK');
catch e; fprintf('  FAIL: %s\n',e.message(1:min(120,end))); end

% Test scenario.plot then updatePlots
disp('Test C: plot(scenario) then advance + updatePlots...');
try
    fig2 = figure('Visible','on');
    plot(scenario);
    advance(scenario);
    scenario.updatePlots();
    disp('  updatePlots OK');
catch e; fprintf('  FAIL: %s\n',e.message(1:min(120,end))); end

% Test chasePlot then advance + updatePlots
disp('Test D: chasePlot(ego) then advance + updatePlots...');
try
    restart(scenario);
    fig3 = figure('Visible','on');
    chasePlot(ego);
    advance(scenario);
    scenario.updatePlots();
    disp('  chasePlot+updatePlots OK');
catch e; fprintf('  FAIL: %s\n',e.message(1:min(120,end))); end

close all;
disp('=== Done ===');
exit;
