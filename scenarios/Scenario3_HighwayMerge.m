function metrics = Scenario3_HighwayMerge(mlModels)
% SCENARIO 3: Highway Merge — Fixed motion + rich moving obstacles
% Ego merges from slip road onto 3-lane highway with fast traffic.
if nargin < 1; mlModels = struct('yolo',[],'deeplab',[],'cnn',[]); end
addpath(fullfile(fileparts(mfilename('fullpath')),'..','utils'));
fprintf('\n[S3] Starting Scenario 3: Highway Merge...\n');
fprintf('[S3] Watch for: overtaking trucks, fast cars, merging bikes, bus\n');

%% ── Build scenario ──────────────────────────────────────────────────────
scenario = drivingScenario('SampleTime',0.05,'StopTime',45);
% 3-lane highway + slip road
road(scenario, [0 0 0; 500 0 0], 'Lanes', lanespec(3));
road(scenario, [-20 -15 0; 30 -6 0; 80 0 0], 'Lanes', lanespec(1,'Width',4));

%% ── Ego vehicle ─────────────────────────────────────────────────────────
ego = vehicle(scenario,'ClassID',1,'Length',4.2,'Width',1.8,'Height',1.5,...
              'PlotColor',[0 0.85 0.3]);
egoID = ego.ActorID;
wayPoints = [-18 -15 0; 30 -6 0; 80 -1.75 0; 150 1.75 0; 300 1.75 0; 490 1.75 0];
nominalSpeed = 18;
% --- Custom Kinematic Initialization ---
ego.Position = wayPoints(1, 1:3);
ego.Velocity = [0 0 0];
ego.Yaw = 0;

currentWaypointIndex = 2;
isHalted = false;
latState = [];

%% ── Moving obstacles ────────────────────────────────────────────────────
% 1. Slow truck 1 in left lane (ego must overtake)
truck1 = vehicle(scenario,'ClassID',2,'Length',14,'Width',2.5,'Height',4.0,...
                 'PlotColor',[0.6 0.6 0.7]);
trajectory(truck1, [80 -5.25 0; 490 -5.25 0], [0; 51.25]);  % 410m/8m/s≈51s
% 2. Slow truck 2 following truck1
truck2 = vehicle(scenario,'ClassID',2,'Length',14,'Width',2.5,'Height',4.0,...
                 'PlotColor',[0.5 0.5 0.6]);
trajectory(truck2, [50 -5.25 0; 490 -5.25 0], [0; 55]);      % 440m/8m/s=55s
% 3. Fast car overtaking in middle lane
fastCar1 = vehicle(scenario,'ClassID',1,'Length',4.0,'Width',1.7,'Height',1.5,...
                   'PlotColor',[1 0.2 0.2]);
trajectory(fastCar1, [30 1.75 0; 490 1.75 0], [0; 19.2]);  % 460m/24m/s≈19s
% 4. Second fast car starting slightly behind
fastCar2 = vehicle(scenario,'ClassID',1,'Length',4.0,'Width',1.7,'Height',1.5,...
                   'PlotColor',[1 0.5 0.1]);
trajectory(fastCar2, [10 1.75 0; 490 1.75 0], [0; 21.8]);  % 480m/22m/s≈21s
% 5. Bus in right lane going same direction
bus = vehicle(scenario,'ClassID',2,'Length',12,'Width',2.5,'Height',3.5,...
              'PlotColor',[0.3 0.5 1]);
trajectory(bus, [0 5.25 0; 490 5.25 0], [0; 40.8]);        % 490m/12m/s≈40s
% 6. Motorbike weaving — actor() because ClassID=3 not valid for vehicle()
moto = actor(scenario,'ClassID',3,'Length',2.2,'Width',0.7,'Height',1.3,...
               'PlotColor',[0.9 0.3 1]);
trajectory(moto, [60 -1.75 0; 200 -1.75 0; 300 1.75 0; 490 1.75 0], [0; 10; 16; 28]);
% 7. Oncoming car in opposite lane (road divider scenario)
oncomingCar = vehicle(scenario,'ClassID',1,'Length',4.0,'Width',1.7,'Height',1.5,...
                      'PlotColor',[0.2 0.9 1]);
trajectory(oncomingCar, [490 -8.75 0; 0 -8.75 0], [0; 24.5]);  % 490m/20m/s
% 8. Debris/stopped car on shoulder
stoppedCar = vehicle(scenario,'ClassID',1,'Length',4.0,'Width',1.7,'Height',1.5,...
                     'PlotColor',[0.7 0.7 0.7]);
trajectory(stoppedCar, [240 5.25 0; 240.01 5.25 0], [0; 60]);   % stationary

%% ── Figures ─────────────────────────────────────────────────────────────
fig1 = figure('Name','S3:Highway-Chase','NumberTitle','off',...
              'Color',[0.04 0.06 0.12],'Position',[30 380 700 400]);
ax1  = axes(fig1,'Color',[0.04 0.06 0.12]);
chasePlot(ego,'Parent',ax1,'Centerline','on');
title(ax1,'[S3] Highway Merge — Chase View (GREEN = EGO)',...
      'Color','w','FontSize',10,'FontWeight','bold');

fig2 = figure('Name','S3:Highway-Top','NumberTitle','off',...
              'Color',[0.04 0.06 0.12],'Position',[740 380 700 400]);
ax2  = axes(fig2,'Color',[0.08 0.1 0.2],'XColor','w','YColor','w');
xlabel(ax2,'X (m)','Color','w'); ylabel(ax2,'Y (m)','Color','w');
hold(ax2,'on');

patch(ax2,[80 280 280 80],[-9 -9 9 9],[0.4 0.9 1],...
      'FaceAlpha',0.07,'EdgeColor',[0.4 0.9 1],'LineWidth',1);
text(ax2,180,7,' MERGE + OVERTAKE ZONE ','Color',[0.4 0.9 1],...
     'FontSize',8,'FontWeight','bold','HorizontalAlignment','center');
plot(ax2, wayPoints(:,1), wayPoints(:,2),'w--','LineWidth',1.5,'DisplayName','Ego path');

egoMk    = plot(ax2,-18,-15,'gs','MarkerSize',12,'MarkerFaceColor','g','DisplayName','EGO');
t1Mk     = plot(ax2, 80,-5.25,'s','MarkerSize', 9,'MarkerFaceColor',[0.6 0.6 0.7],'MarkerEdgeColor','w','DisplayName','Truck1');
t2Mk     = plot(ax2, 50,-5.25,'s','MarkerSize', 9,'MarkerFaceColor',[0.5 0.5 0.6],'MarkerEdgeColor','w','DisplayName','Truck2');
fc1Mk    = plot(ax2, 30, 1.75,'rs','MarkerSize', 8,'MarkerFaceColor','r','DisplayName','FastCar1');
motoMk   = plot(ax2, 60,-1.75,'ms','MarkerSize', 8,'MarkerFaceColor','m','DisplayName','Moto');

legend(ax2,'show','TextColor','w','Color',[0.08 0.1 0.2],'FontSize',7,'Location','northeast');
title(ax2,'[S3] Highway Top View (LIVE) — 500m road','Color','w','FontSize',9,'FontWeight','bold');
xlim(ax2,[-25 500]); ylim(ax2,[-20 12]); grid(ax2,'on'); axis(ax2,'manual');

%% ── Simulation loop ──────────────────────────────────────────────────────
metrics    = MetricsLogger('Scenario3_HighwayMerge');
FRAME_SIZE = [120 160 3];
mlLog.yoloTrucks = 0; stepN = 0; trajX = []; trajY = [];

% FIX: Pre-allocate trajectory line for real-time update
trajLine = plot(ax2, NaN, NaN, 'c-', 'LineWidth', 2.5, 'Tag', 'tr');

fprintf('[S3] Simulation running...\n');
tLoop = tic;
while advance(scenario)
    stepN = stepN + 1;
    egoPos   = ego.Position;
    egoVel   = ego.Velocity;
    currSpeed= norm(egoVel(1:2));
    if currSpeed > 0.01
        egoYaw = atan2(egoVel(2), egoVel(1)+1e-9);
    else
        egoYaw = ego.Yaw * pi/180;
    end
    
    ap = actorPoses(scenario);
    aPoses = struct('Position',{}, 'Width',{}, 'Length',{}); idx = 0;
    t1P=[80,-5.25]; t2P=[50,-5.25]; fc1P=[30,1.75]; mP=[60,-1.75];
    
    for k = 1:numel(ap)
        aID = scenario.Actors(k).ActorID;
        if aID ~= egoID
            idx = idx+1;
            aPoses(idx).Position = ap(k).Position;
            aPoses(idx).Width = scenario.Actors(k).Width;
            aPoses(idx).Length = scenario.Actors(k).Length;
            if aID == truck1.ActorID;  t1P = ap(k).Position(1:2); end
            if aID == truck2.ActorID;  t2P = ap(k).Position(1:2); end
            if aID == fastCar1.ActorID;fc1P= ap(k).Position(1:2); end
            if aID == moto.ActorID;    mP  = ap(k).Position(1:2); end
        end
    end
    
    % --- DYNAMIC TRAJECTORY REPLANNING (PlannerBridge) ---
    if exist('wayPoints', 'var'); wps = wayPoints; elseif exist('wpNormal', 'var'); wps = wpNormal; else; wps = []; end
    if ~exist('latState', 'var'); latState = []; end
    
    [safeVel, shiftedWP, status, minDist, latState] = PlannerBridge(wps, egoPos, egoYaw, aPoses, egoID, nominalSpeed, latState);
    
    inOT = egoPos(1) >= 80 && egoPos(1) <= 280;
    if inOT && strcmp(status,'CRUISE'); status = 'OVERTAKING'; end
    
    frame = SyntheticCamera(scenario, egoPos, FRAME_SIZE(1:2));
    [~,~,~,det] = RunYOLO(frame, mlModels.yolo, 0.80);
    if det.closestDist < 25 && inOT
        status = 'YOLO-BLOCKED'; mlLog.yoloTrucks = mlLog.yoloTrucks + 1;
    end
    
    [~,cnnSpd,cnnConf] = RunCNNControl(frame, mlModels.cnn, FRAME_SIZE, nominalSpeed);
    if cnnConf > 0.5
        v_final = 0.35*cnnSpd + 0.65*currSpeed;
    else
        v_final = currSpeed;
    end
    
    % --- BEGIN ADVANCED COLLISION & HALT LOGIC ---
    % --- 1. Custom Kinematic Waypoint Follower (with Lookahead) ---
    if ~isempty(shiftedWP) && currentWaypointIndex <= size(shiftedWP, 1)
        % Advance currentWaypointIndex if we passed it
        targetWp = shiftedWP(currentWaypointIndex, 1:3);
        vecToWp = targetWp(1:2) - ego.Position(1:2);
        if dot(vecToWp, [cosd(ego.Yaw), sind(ego.Yaw)]) < 0
            currentWaypointIndex = min(currentWaypointIndex + 1, size(shiftedWP, 1));
        end
        
        % Find a lookahead point at least 5m ahead to prevent violent steering
        lookaheadWp = targetWp;
        lookIdx = currentWaypointIndex;
        while norm(lookaheadWp(1:2) - ego.Position(1:2)) < 5.0 && lookIdx < size(shiftedWP, 1)
            lookIdx = lookIdx + 1;
            lookaheadWp = shiftedWP(lookIdx, 1:3);
        end
        
        vecLook = lookaheadWp(1:2) - ego.Position(1:2);
        
        % Smooth yaw change (max 15 deg per frame) to prevent instant spin crashes
        targetYaw = atan2d(vecLook(2), vecLook(1));
        yawDiff = wrapTo180(targetYaw - ego.Yaw);
        ego.Yaw = ego.Yaw + sign(yawDiff) * min(abs(yawDiff), 15.0);
    end
    
    % --- 2. Dynamic Stop & Go Logic ---
    if strcmp(status, 'STOPPED') || strcmp(status, 'LATERAL-BLOCKED') || (strcmp(status, 'BRAKING') && minDist < 2.0)
        isHalted = true;
    else
        isHalted = false;
    end
    
    % --- 3. Kinematic Execution ---
    if isHalted || (~isempty(shiftedWP) && currentWaypointIndex > size(shiftedWP, 1))
        ego.Velocity = [0, 0, 0];
    else
        ego.Velocity = [safeVel * cosd(ego.Yaw), safeVel * sind(ego.Yaw), 0];
    end
    
    ego.Position = ego.Position + ego.Velocity * scenario.SampleTime;
    % ── ROAD BOUNDARY CLAMP (Dynamic — follows curved road) ───────────────
    ROAD_HALF_WIDTH = 3.5;
    roadCentreY = interp1(wayPoints(:,1), wayPoints(:,2), ego.Position(1), 'linear', 'extrap');
    ego.Position(2) = max(roadCentreY - ROAD_HALF_WIDTH, min(roadCentreY + ROAD_HALF_WIDTH, ego.Position(2)));
    if ego.Position(2) <= roadCentreY - ROAD_HALF_WIDTH + 0.3 && ego.Yaw < -5; ego.Yaw = max(ego.Yaw, -5); end
    if ego.Position(2) >= roadCentreY + ROAD_HALF_WIDTH - 0.3 && ego.Yaw >  5; ego.Yaw = min(ego.Yaw,  5); end
    % ───────────────────────────────────────────────────────────────────
    
    egoPos = ego.Position;
    currSpeed = norm(ego.Velocity);
    egoYaw = ego.Yaw * pi/180;
    
    dt = scenario.SampleTime;
    crashDetected = false;
    for actIdx = 1:numel(scenario.Actors)
        act = scenario.Actors(actIdx);
        if act.ActorID == ego.ActorID; continue; end
        
        % OBB / AABB Approximation in Ego Frame
        dx = act.Position(1) - egoPos(1);
        dy = act.Position(2) - egoPos(2);
        
        cosY = cos(egoYaw); sinY = sin(egoYaw);
        localX = dx * cosY + dy * sinY;
        localY = -dx * sinY + dy * cosY;
        
        clearanceX = abs(localX) - (ego.Length/2 + act.Length/2);
        clearanceY = abs(localY) - (ego.Width/2 + act.Width/2);
        
        relSpeed = norm(egoVel(1:2) - act.Velocity(1:2));
        longitudinalBuffer = max(relSpeed * dt, 0.1);
        lateralBuffer = 0.1;
        
        if clearanceX <= longitudinalBuffer && clearanceY <= lateralBuffer
            metrics.CollisionOccurred = true;
            fprintf('💥 [CRASH] Physical collision with Actor %d at T=%.2fs!\n', act.ActorID, scenario.SimulationTime);
            crashDetected = true;
            break;
        end
    end
    
    if crashDetected
        break; % Break the while loop
    end
    % --- END ADVANCED COLLISION & HALT LOGIC ---
    
    metrics = LogStep(metrics, currSpeed, egoYaw, minDist, status);
    trajX(end+1) = egoPos(1);
    trajY(end+1) = egoPos(2);
    
    c = sc(status);
    try
        title(ax1, sprintf('[S3] x=%.0fm  %.1fm/s (%.0fkm/h) | %s | YOLO:%d',...
            egoPos(1), currSpeed, currSpeed*3.6, status, mlLog.yoloTrucks),...
            'Color',c,'FontSize',9,'FontWeight','bold');
    catch; end
    
    try
        % FIX: Update XData/YData instead of destroying and recreating the plot
        set(trajLine, 'XData', trajX, 'YData', trajY);
        set(egoMk,  'XData',egoPos(1), 'YData',egoPos(2));
        set(t1Mk,   'XData',t1P(1),    'YData',t1P(2));
        set(t2Mk,   'XData',t2P(1),    'YData',t2P(2));
        set(fc1Mk,  'XData',fc1P(1),   'YData',fc1P(2));
        set(motoMk, 'XData',mP(1),     'YData',mP(2));
    catch; end
    
    scenario.updatePlots();
    drawnow limitrate;
    
    % FIX: Real-time pacing (1x speed instead of 10x)
    elapsed = toc(tLoop);
    targetT = stepN * scenario.SampleTime;
    if elapsed < targetT; pause(targetT - elapsed); end
    
    if egoPos(1) >= 487
        metrics.GoalReached = true;
        fprintf('[S3] GOAL! x=%.0f YOLO-blocks=%d\n', egoPos(1), mlLog.yoloTrucks);
        break;
    end
end
PrintSummary(metrics);
fprintf('[S3] Scenario 3 Complete.\n');
end

function c = sc(s)
    if contains(s,'LANE-CHANGE'); c=[0.2 0.9 1]; return; end
    switch s
        case 'CRUISE';       c=[0.2 0.9 0.4];
        case 'CAUTION';      c=[1.0 0.8 0.1];
        case 'BRAKING';      c=[1.0 0.4 0.1];
        case 'STOPPED';      c=[1.0 0.1 0.1];
        case 'OVERTAKING';   c=[0.4 0.9 1.0];
        case 'YOLO-BLOCKED'; c=[1.0 0.5 0.1];
        otherwise;           c=[1.0 1.0 1.0];
    end
end