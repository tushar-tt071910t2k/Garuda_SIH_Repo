% ProbeActorPoses.m - Find correct way to get trajectory-actor positions
disp('=== Probing Actor Pose API ===');
scenario = drivingScenario('SampleTime',0.1,'StopTime',5);
road(scenario,[0 0 0; 50 0 0]);
ego = vehicle(scenario,'ClassID',1);
smoothTrajectory(ego,[0 0 0; 48 0 0],5);
ped = actor(scenario,'ClassID',4,'Length',0.5,'Width',0.5,'Height',1.7);
trajectory(ped,[5 -3 0; 5 3 0],1.5);
debris = actor(scenario,'ClassID',5,'Length',1,'Width',0.8,'Height',0.3);
trajectory(debris,[20 0 0; 20.01 0 0],0.001);

advance(scenario);

% Test 1: ego.state (smoothTrajectory)
disp('Test 1: ego.state (smoothTrajectory actor)...');
try; s=ego.state; fprintf('  OK: pos=[%.1f %.1f %.1f]\n',s.Position); catch e; fprintf('  FAIL: %s\n',e.message(1:80)); end

% Test 2: ped.state (trajectory actor) - expect fail
disp('Test 2: ped.state (trajectory actor)...');
try; s=ped.state; fprintf('  OK: pos=[%.1f %.1f %.1f]\n',s.Position); catch e; fprintf('  FAIL: %s\n',e.message(1:80)); end

% Test 3: targetPoses(ego)
disp('Test 3: targetPoses(ego)...');
try
    tp=targetPoses(ego);
    fprintf('  OK: %d actors returned\n',numel(tp));
    if numel(tp)>0
        fprintf('  Actor 1: pos=[%.1f %.1f %.1f]  ClassID=%d\n',...
            tp(1).Position,tp(1).ClassID);
    end
    disp('  Fields:'); disp(fieldnames(tp(1))');
catch e; fprintf('  FAIL: %s\n',e.message(1:80)); end

% Test 4: actorPoses(scenario)
disp('Test 4: actorPoses(scenario)...');
try
    ap=actorPoses(scenario);
    fprintf('  OK: %d actors\n',numel(ap));
    if numel(ap)>0; fprintf('  Actor 1 pos=[%.1f %.1f %.1f]\n',ap(1).Position); end
    disp('  Fields:'); disp(fieldnames(ap(1))');
catch e; fprintf('  FAIL: %s\n',e.message(1:80)); end

% Test 5: scenario.actorStates
disp('Test 5: actorStates(scenario)...');
try
    as=actorStates(scenario);
    fprintf('  OK: class=%s size=%s\n',class(as),mat2str(size(as)));
    disp(as(1));
catch e; fprintf('  FAIL: %s\n',e.message(1:80)); end

disp('=== Done ===');
exit;
