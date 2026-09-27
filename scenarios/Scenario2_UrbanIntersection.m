function metrics = Scenario2_UrbanIntersection(mlModels)
% SCENARIO 2: Busy Urban Intersection — Fixed motion + rich moving obstacles
% Ego crosses a 4-way intersection with heavy cross-traffic.
if nargin < 1; mlModels = struct('yolo',[],'deeplab',[],'cnn',[]); end
addpath(fullfile(fileparts(mfilename('fullpath')),'..','utils'));
fprintf('\n[S2] Starting Scenario 2: Busy Urban Intersection...\n');
fprintf('[S2] Watch for: cross cars, motorbike, pedestrians, bus, auto-rickshaw\n');

%% ── Build scenario ──────────────────────────────────────────────────────
scenario = drivingScenario('SampleTime',0.05,'StopTime',60);
% Two crossing roads (4-way intersection)
road(scenario, [-80 0 0; 80 0 0], 'Lanes', lanespec(2));
road(scenario, [0 -80 0;  0 80 0], 'Lanes', lanespec(2));

%% ── Ego vehicle: approaches and crosses intersection ────────────────────
ego = vehicle(scenario,'ClassID',1,'Length',4.2,'Width',1.8,'Height',1.5,...
              'PlotColor',[0 0.8 0.3]);
egoID = ego.ActorID;
wayPoints = [-75 -1.75 0; -20 -1.75 0; 0 -1.75 0; 20 -1.75 0; 75 -1.75 0];
nominalSpeed = 7;
% --- Custom Kinematic Initialization ---
ego.Position = wayPoints(1, 1:3);
ego.Velocity = [0 0 0];
ego.Yaw = 0;

currentWaypointIndex = 2;
isHalted = false;
latState = [];
yieldTimer = 0;   % counts frames ego is halted at yield zone
YIELD_MAX_FRAMES = 160;  % 8 s at 0.05 s/frame — then creep through

%% ── Moving obstacles ────────────────────────────────────────────────────
% 1. Auto-rickshaw — actor() because ClassID=7 not valid for vehicle()
auto1 = actor(scenario,'ClassID',7,'Length',2.8,'Width',1.3,'Height',1.7,...
                'PlotColor',[1 0.6 0]);
trajectory(auto1, [2 -70 0; 2 -5 0; 2 5 0; 2 70 0], [0; 8; 10; 18]);
% 2. Oncoming car (right lane, heading towards ego)
car1 = vehicle(scenario,'ClassID',1,'Length',4.0,'Width',1.7,'Height',1.5,...
               'PlotColor',[0.8 0.1 0.1]);
trajectory(car1, [72 1.75 0; 5 1.75 0; -72 1.75 0], [0; 7.4; 16.2]);  % 9m/s
% 3. Motorbike — actor() because ClassID=3 not valid for vehicle()
bike1 = actor(scenario,'ClassID',3,'Length',2.0,'Width',0.7,'Height',1.2,...
                'PlotColor',[0.9 0.2 0.9]);
trajectory(bike1, [-2 -75 0; -2 -3 0; -2 3 0; -2 75 0], [0; 10; 12; 22]);
% 4. Big bus crossing N→S slowly
bus = vehicle(scenario,'ClassID',2,'Length',12,'Width',2.5,'Height',3.5,...
              'PlotColor',[0.3 0.5 1]);
trajectory(bus, [-5 -80 0; -5 -18 0; -5 -5 0; -5 80 0], [0; 8; 14; 28]);
% 5. Pedestrian 1 — starts off-road, approaches then crosses at t=5
ped1 = actor(scenario,'ClassID',4,'Length',0.5,'Width',0.5,'Height',1.7,...
             'PlotColor',[1 0.4 0.2]);
trajectory(ped1, [-30 -18 0; -30 -8 0; -30 6 0], [0; 5; 11]);
% 6. Pedestrian 2 — starts off-road, approaches then crosses at t=7
ped2 = actor(scenario,'ClassID',4,'Length',0.5,'Width',0.5,'Height',1.7,...
             'PlotColor',[1 0.8 0.1]);
trajectory(ped2, [35 18 0; 35 8 0; 35 -6 0], [0; 7; 14]);
% 7. Parked truck partially blocking left lane at x=-55
parkedTruck = vehicle(scenario,'ClassID',2,'Length',10,'Width',2.4,'Height',3.5,...
                      'PlotColor',[0.5 0.5 0.6]);
trajectory(parkedTruck, [-55 -0.5 0; -54.99 -0.5 0], [0; 60]);   % stationary
% 8. Second car crossing through intersection later
car2 = vehicle(scenario,'ClassID',1,'Length',4.0,'Width',1.7,'Height',1.5,...
               'PlotColor',[0.2 0.8 1]);
trajectory(car2, [4 75 0; 4 8 0; 4 -8 0; 4 -75 0], [0; 10; 14; 25]);

%% ── Figures ─────────────────────────────────────────────────────────────
fig1 = figure('Name','S2:Intersection-Chase','NumberTitle','off',...
              'Color',[0.06 0.08 0.14],'Position',[30 380 700 400]);
ax1  = axes(fig1,'Color',[0.06 0.08 0.14]);
chasePlot(ego,'Parent',ax1,'Centerline','on');
title(ax1,'[S2] Urban Intersection — Chase View',...
      'Color','w','FontSize',10,'FontWeight','bold');

fig2 = figure('Name','S2:Intersection-Top','NumberTitle','off',...
              'Color',[0.06 0.08 0.14],'Position',[740 380 700 400]);
ax2  = axes(fig2,'Color',[0.1 0.1 0.18],'XColor','w','YColor','w');
xlabel(ax2,'X (m)','Color','w'); ylabel(ax2,'Y (m)','Color','w');
hold(ax2,'on');

% Intersection zone highlight
patch(ax2,[-20 20 20 -20],[-8 -8 8 8],[1 0.85 0.1],...
      'FaceAlpha',0.12,'EdgeColor',[1 0.85 0.1],'LineWidth',1.5);
text(ax2, 0, 9,'  INTERSECTION  ','Color',[1 0.85 0.1],...
     'FontSize',9,'FontWeight','bold','HorizontalAlignment','center');
plot(ax2, wayPoints(:,1), wayPoints(:,2),'w--','LineWidth',1.5,'DisplayName','Ego path');

% Moving actor markers
egoMk  = plot(ax2,-75,-1.75,'gs','MarkerSize',12,'MarkerFaceColor','g','DisplayName','EGO');
autoMk = plot(ax2,  2,-70,  's','MarkerSize', 8,'MarkerFaceColor',[1 0.6 0],'DisplayName','Auto');
bikeMk = plot(ax2, -2,-75,  'ms','MarkerSize', 8,'MarkerFaceColor','m','DisplayName','Bike');
busMk  = plot(ax2, -5,-80,  'bs','MarkerSize',10,'MarkerFaceColor','b','DisplayName','Bus');
car1Mk = plot(ax2, 72, 1.75,'rs','MarkerSize', 8,'MarkerFaceColor','r','DisplayName','Car1');
legend(ax2,'show','TextColor','w','Color',[0.1 0.1 0.18],'FontSize',7,'Location','southeast');
title(ax2,'[S2] Intersection Top View (LIVE) — yellow zone = yield','Color','w','FontSize',9,'FontWeight','bold');
xlim(ax2,[-85 85]); ylim(ax2,[-85 85]); grid(ax2,'on'); axis(ax2,'manual');

%% ── Simulation loop ──────────────────────────────────────────────────────
metrics   = MetricsLogger('Scenario2_UrbanIntersection');
FRAME_SIZE = [120 160 3];
mlLog.yoloHits = 0; stepN = 0; trajX = []; trajY = [];

% FIX: Pre-allocate trajectory line for real-time update
trajLine = plot(ax2, NaN, NaN, 'c-', 'LineWidth', 2.5, 'Tag', 'tr');

fprintf('[S2] Simulation running...\n');
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
    
    % Gather live actor positions for HUD
    ap = actorPoses(scenario);
    aPoses = struct('Position',{}, 'Width',{}, 'Length',{}); idx = 0;
    aPos=[2,-70]; bPos=[-2,-75]; busPos=[-5,-80]; c1Pos=[72,1.75];
    
    for k = 1:numel(ap)
        aID = scenario.Actors(k).ActorID;
        if aID ~= egoID
            idx = idx+1;
            aPoses(idx).Position = ap(k).Position;
            aPoses(idx).Width = scenario.Actors(k).Width;
            aPoses(idx).Length = scenario.Actors(k).Length;
            if aID == auto1.ActorID;  aPos  = ap(k).Position(1:2); end
            if aID == bike1.ActorID;  bPos  = ap(k).Position(1:2); end
            if aID == bus.ActorID;    busPos= ap(k).Position(1:2); end
            if aID == car1.ActorID;   c1Pos = ap(k).Position(1:2); end
        end
    end
    
    % --- DYNAMIC TRAJECTORY REPLANNING (PlannerBridge) ---
    if exist('wayPoints', 'var'); wps = wayPoints; elseif exist('wpNormal', 'var'); wps = wpNormal; else; wps = []; end
    if ~exist('latState', 'var'); latState = []; end
    
    [safeVel, shiftedWP, status, minDist, latState] = PlannerBridge(wps, egoPos, egoYaw, aPoses, egoID, nominalSpeed, latState);
    
    % Yield zone logic (display only)
    inYield = abs(egoPos(1)) <= 20;
    if inYield && strcmp(status,'CRUISE'); status = 'YIELDING'; end
    
    % ML stubs
    frame = SyntheticCamera(scenario, egoPos, FRAME_SIZE(1:2));
    [~,~,~,det] = RunYOLO(frame, mlModels.yolo, 0.80);
    if (det.hasPerson || det.hasAuto) && det.closestDist < 15 && inYield
        status = 'YOLO-YIELD'; mlLog.yoloHits = mlLog.yoloHits + 1;
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
    if strcmp(status, 'STOPPED') || strcmp(status, 'LATERAL-BLOCKED') || ...
       (strcmp(status, 'BRAKING') && minDist < 2.0) || strcmp(status, 'YOLO-YIELD')
        isHalted = true;
        yieldTimer = yieldTimer + 1;
    else
        isHalted = false;
        yieldTimer = 0;
    end
    % YIELD TIMEOUT: if stuck at intersection > 8s, creep through at 2 m/s
    if yieldTimer > YIELD_MAX_FRAMES
        isHalted = false;
        safeVel   = 2.0;
        status    = 'CREEP-YIELD';
    end
    
    % --- 3. Kinematic Execution ---
    if isHalted || (~isempty(shiftedWP) && currentWaypointIndex > size(shiftedWP, 1))
        ego.Velocity = [0, 0, 0];
    else
        ego.Velocity = [safeVel * cosd(ego.Yaw), safeVel * sind(ego.Yaw), 0];
    end
    
    ego.Position = ego.Position + ego.Velocity * scenario.SampleTime;
    % ── ROAD BOUNDARY CLAMP ──────────────────────────────────────────────────
    % 2-lane road: 2 x 3.6m = 7.2m wide, keep ego within ±3.6m of centre.
    ego.Position(2) = max(-3.6, min(3.6, ego.Position(2)));
    % Yaw correction: if pressing against boundary, stop steering into wall
    if ego.Position(2) <= -3.4 && ego.Yaw < -5; ego.Yaw = max(ego.Yaw, -5); end
    if ego.Position(2) >=  3.4 && ego.Yaw >  5; ego.Yaw = min(ego.Yaw,  5); end
    % ─────────────────────────────────────────────────────────────────────────
    
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
        title(ax1, sprintf('[S2] x=%.1fm  spd=%.1fm/s | %s | YOLO:%d',...
            egoPos(1), currSpeed, status, mlLog.yoloHits),...
            'Color',c,'FontSize',9,'FontWeight','bold');
    catch; end
    
    try
        % FIX: Update XData/YData instead of destroying and recreating the plot
        set(trajLine, 'XData', trajX, 'YData', trajY);
        set(egoMk,  'XData',egoPos(1), 'YData',egoPos(2));
        set(autoMk, 'XData',aPos(1),   'YData',aPos(2));
        set(bikeMk, 'XData',bPos(1),   'YData',bPos(2));
        set(busMk,  'XData',busPos(1), 'YData',busPos(2));
        set(car1Mk, 'XData',c1Pos(1),  'YData',c1Pos(2));
    catch; end
    
    scenario.updatePlots();
    drawnow limitrate;
    
    % FIX: Real-time pacing (1x speed instead of 10x)
    elapsed = toc(tLoop);
    targetT = stepN * scenario.SampleTime;
    if elapsed < targetT; pause(targetT - elapsed); end
    
    if egoPos(1) >= 73
        metrics.GoalReached = true;
        fprintf('[S2] GOAL! x=%.1f YOLO=%d\n', egoPos(1), mlLog.yoloHits);
        break;
    end
end
PrintSummary(metrics);
fprintf('[S2] Scenario 2 Complete.\n');
end

function c = sc(s)
    if contains(s,'LATERAL'); c=[0.3 0.8 1.0]; return; end
    switch s
        case 'CRUISE';     c=[0.2 0.9 0.4];
        case 'CAUTION';    c=[1.0 0.8 0.1];
        case 'BRAKING';    c=[1.0 0.4 0.1];
        case 'STOPPED';    c=[1.0 0.1 0.1];
        case 'YIELDING';   c=[0.6 0.4 1.0];
        case 'YOLO-YIELD'; c=[0.9 0.3 0.9];
        otherwise;         c=[1.0 1.0 1.0];
    end
end