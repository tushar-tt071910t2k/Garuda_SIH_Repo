function metrics = Scenario5_CattleCrossing(mlModels)
% SCENARIO 5: Cattle Crossing — Fixed motion + rich moving obstacles
% Ego navigates 250m road, encounters cattle herd, takes detour.
if nargin < 1; mlModels = struct('yolo',[],'deeplab',[],'cnn',[]); end
addpath(fullfile(fileparts(mfilename('fullpath')),'..','utils'));
fprintf('\n[S5] Starting Scenario 5: Cattle Crossing...\n');
fprintf('[S5] Watch: 4 cows crossing, shepherd, bikes, oncoming truck, detour\n');

%% ── Build scenario ──────────────────────────────────────────────────────
scenario = drivingScenario('SampleTime',0.05,'StopTime',90);
road(scenario, [0 0 0; 250 0 0], 'Lanes', lanespec(2));

%% ── Ego vehicle ─────────────────────────────────────────────────────────
ego = vehicle(scenario,'ClassID',1,'Length',4.2,'Width',1.8,'Height',1.5,...
              'PlotColor',[0 0.85 0.3]);
egoID = ego.ActorID;
% Normal path — ego will follow this (cattle zone x=95–130)
wpNormal = [0 -1.75 0; 80 -1.75 0; 115 0 0; 145 1.75 0; 200 1.75 0; 245 1.75 0];
nominalSpeed = 10;
% --- Custom Kinematic Initialization ---
ego.Position = wpNormal(1, 1:3);
ego.Velocity = [0 0 0];
ego.Yaw = 0;

currentWaypointIndex = 2;
isHalted = false;
latState = [];

%% ── Cattle herd (4 cows) crossing road at different times ───────────────
% Cow 1: approaches from side, then crosses left→right
cow1 = actor(scenario,'ClassID',5,'Length',2.0,'Width',0.7,'Height',1.3,...
             'PlotColor',[0.55 0.35 0.15]);
trajectory(cow1, [100 -14 0; 100 -6 0; 100 4 0], [0; 4; 10]);
% Cow 2: approaches from other side, crosses right→left
cow2 = actor(scenario,'ClassID',5,'Length',2.0,'Width',0.7,'Height',1.3,...
             'PlotColor',[0.65 0.40 0.20]);
trajectory(cow2, [112 14 0; 112 6 0; 112 -4 0], [0; 5; 12]);
% Cow 3: starts off-road, then meanders across
cow3 = actor(scenario,'ClassID',5,'Length',2.0,'Width',0.7,'Height',1.3,...
             'PlotColor',[0.50 0.30 0.10]);
trajectory(cow3, [108 -13 0; 108 -5 0; 108 -1 0; 110 5 0], [0; 6; 10; 16]);
% Cow 4: starts off-road, grazes then retreats
cow4 = actor(scenario,'ClassID',5,'Length',2.0,'Width',0.7,'Height',1.3,...
             'PlotColor',[0.60 0.38 0.18]);
trajectory(cow4, [120 10 0; 120 2 0; 115 -3 0; 105 -5 0], [0; 7; 13; 20]);

%% ── Shepherd crossing ────────────────────────────────────────────────────
% Shepherd: starts off-road, approaches then herds cattle across
shepherd = actor(scenario,'ClassID',4,'Length',0.5,'Width',0.5,'Height',1.8,...
                 'PlotColor',[0.9 0.7 0.4]);
trajectory(shepherd, [115 -18 0; 115 -8 0; 115 7 0], [0; 3; 12]);

%% ── Other road actors ────────────────────────────────────────────────────
% Oncoming truck (opposite lane)
truck = vehicle(scenario,'ClassID',2,'Length',14,'Width',2.5,'Height',4.0,...
                'PlotColor',[0.4 0.4 0.6]);
trajectory(truck, [245 1.75 0; 0 1.75 0], [0; 28]);   % 245m / 9m/s ≈ 27s
% Bike 1 — actor() because ClassID=3 not valid for vehicle()
bike1 = actor(scenario,'ClassID',3,'Length',2.0,'Width',0.7,'Height',1.2,...
                'PlotColor',[1 0.2 0.8]);
trajectory(bike1, [0 -3.5 0; 240 -3.5 0], [0; 17.1]);  % 240m/14m/s≈17s
% Bike 2 — actor() because ClassID=3 not valid for vehicle()
bike2 = actor(scenario,'ClassID',3,'Length',2.0,'Width',0.7,'Height',1.2,...
                'PlotColor',[0.2 0.8 1]);
trajectory(bike2, [240 1.75 0; 0 1.75 0], [0; 20]);    % 240m/12m/s=20s
% Stopped vehicle on shoulder at x=75 (breakdwon)
brokenCar = vehicle(scenario,'ClassID',1,'Length',4.0,'Width',1.7,'Height',1.5,...
                    'PlotColor',[0.7 0.7 0.7]);
trajectory(brokenCar, [75 -4.5 0; 75.01 -4.5 0], [0; 60]);  % stationary

%% ── Figures ─────────────────────────────────────────────────────────────
fig1 = figure('Name','S5:Cattle-Chase','NumberTitle','off',...
              'Color',[0.07 0.10 0.07],'Position',[30 380 700 400]);
ax1  = axes(fig1,'Color',[0.07 0.10 0.07]);
chasePlot(ego,'Parent',ax1,'Centerline','on');
title(ax1,'[S5] Cattle Crossing — Chase View (GREEN = EGO)',...
      'Color','w','FontSize',10,'FontWeight','bold');

fig2 = figure('Name','S5:Cattle-Top','NumberTitle','off',...
              'Color',[0.07 0.10 0.07],'Position',[740 380 700 400]);
ax2  = axes(fig2,'Color',[0.08 0.13 0.08],'XColor','w','YColor','w');
xlabel(ax2,'X (m)','Color','w'); ylabel(ax2,'Y (m)','Color','w');
hold(ax2,'on');

% Cattle zone highlight
patch(ax2,[88 135 135 88],[-8 -8 9 9],[0.8 0.4 0.1],...
      'FaceAlpha',0.12,'EdgeColor',[0.8 0.4 0.1],'LineWidth',1.5);
text(ax2,111, 9.5,'  CATTLE ZONE  ','Color',[1 0.5 0.2],...
     'FontSize',9,'FontWeight','bold','HorizontalAlignment','center');

% Normal + detour paths
plot(ax2, wpNormal(:,1), wpNormal(:,2),'w--','LineWidth',1.5,'DisplayName','Normal path');

% Moving markers
egoMk   = plot(ax2,  0,-1.75,'gs','MarkerSize',12,'MarkerFaceColor','g','DisplayName','EGO');
c1Mk    = plot(ax2,100,-6,   's','MarkerSize',10,'MarkerFaceColor',[0.55 0.35 0.15],'MarkerEdgeColor','w','DisplayName','Cow1');
c2Mk    = plot(ax2,112, 6,   's','MarkerSize',10,'MarkerFaceColor',[0.65 0.40 0.20],'MarkerEdgeColor','w','DisplayName','Cow2');
c3Mk    = plot(ax2,108,-5,   's','MarkerSize',10,'MarkerFaceColor',[0.50 0.30 0.10],'MarkerEdgeColor','w','DisplayName','Cow3');
truckMk = plot(ax2,245, 1.75,'bs','MarkerSize',10,'MarkerFaceColor','b','DisplayName','Truck');
b1Mk    = plot(ax2,  0,-3.5, 'ms','MarkerSize', 8,'MarkerFaceColor','m','DisplayName','Bike1');
legend(ax2,'show','TextColor','w','Color',[0.08 0.13 0.08],'FontSize',7,'Location','northeast');
title(ax2,'[S5] Cattle Crossing Top View (LIVE) — 250m road',...
      'Color','w','FontSize',9,'FontWeight','bold');
xlim(ax2,[-5 252]); ylim(ax2,[-10 12]); grid(ax2,'on'); axis(ax2,'manual');

%% ── Simulation loop ──────────────────────────────────────────────────────
metrics    = MetricsLogger('Scenario5_CattleCrossing');
FRAME_SIZE = [120 160 3];
mlLog.yoloCows = 0; stepN = 0; trajX = []; trajY = [];

% FIX: Pre-allocate trajectory line for real-time update
trajLine = plot(ax2, NaN, NaN, 'c-', 'LineWidth', 2.5, 'Tag', 'tr');

fprintf('[S5] Simulation running...\n');
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
    c1P=[100,-6]; c2P=[112,6]; c3P=[108,-5]; trP=[245,1.75]; b1P=[0,-3.5];
    
    for k = 1:numel(ap)
        aID = scenario.Actors(k).ActorID;
        if aID ~= egoID
            idx = idx+1;
            aPoses(idx).Position = ap(k).Position;
            aPoses(idx).Width = scenario.Actors(k).Width;
            aPoses(idx).Length = scenario.Actors(k).Length;
            if aID == cow1.ActorID;  c1P = ap(k).Position(1:2); end
            if aID == cow2.ActorID;  c2P = ap(k).Position(1:2); end
            if aID == cow3.ActorID;  c3P = ap(k).Position(1:2); end
            if aID == truck.ActorID; trP = ap(k).Position(1:2); end
            if aID == bike1.ActorID; b1P = ap(k).Position(1:2); end
        end
    end
    
    % --- DYNAMIC TRAJECTORY REPLANNING (PlannerBridge) ---
    if exist('wayPoints', 'var'); wps = wayPoints; elseif exist('wpNormal', 'var'); wps = wpNormal; else; wps = []; end
    if ~exist('latState', 'var'); latState = []; end
    
    [safeVel, shiftedWP, status, minDist, latState] = PlannerBridge(wps, egoPos, egoYaw, aPoses, egoID, nominalSpeed, latState);
    
    % Cattle zone display logic
    inCattleZone = egoPos(1) >= 85 && egoPos(1) <= 135;
    if inCattleZone && strcmp(status,'CRUISE'); status = 'CATTLE-ZONE'; end
    
    % ML stubs
    frame = SyntheticCamera(scenario, egoPos, FRAME_SIZE(1:2));
    [~,~,~,det] = RunYOLO(frame, mlModels.yolo, 0.80);
    if det.hasCow && det.closestDist < 20
        status = 'YOLO-COW'; mlLog.yoloCows = mlLog.yoloCows + 1;
    end
    if det.hasPerson && det.closestDist < 12
        status = 'YOLO-SHEPHERD';
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
    roadCentreY = interp1(wpNormal(:,1), wpNormal(:,2), ego.Position(1), 'linear', 'extrap');
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
            fprintf('   -> Ego Pos: [%.2f, %.2f], Yaw: %.2f\n', ego.Position(1), ego.Position(2), ego.Yaw);
            fprintf('   -> Act Pos: [%.2f, %.2f]\n', act.Position(1), act.Position(2));
            fprintf('   -> clearanceX=%.2f, clearanceY=%.2f, longBuf=%.2f, latBuf=%.2f\n', clearanceX, clearanceY, longitudinalBuffer, lateralBuffer);
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
        title(ax1, sprintf('[S5] x=%.0fm  spd=%.1fm/s | %s | minDist:%.1fm | YOLO-cows:%d',...
            egoPos(1), currSpeed, status, minDist, mlLog.yoloCows),...
            'Color',c,'FontSize',9,'FontWeight','bold');
    catch; end
    
    try
        % FIX: Update XData/YData instead of destroying and recreating the plot
        set(trajLine, 'XData', trajX, 'YData', trajY);
        set(egoMk,   'XData',egoPos(1), 'YData',egoPos(2));
        set(c1Mk,    'XData',c1P(1),    'YData',c1P(2));
        set(c2Mk,    'XData',c2P(1),    'YData',c2P(2));
        set(c3Mk,    'XData',c3P(1),    'YData',c3P(2));
        set(truckMk, 'XData',trP(1),    'YData',trP(2));
        set(b1Mk,    'XData',b1P(1),    'YData',b1P(2));
    catch; end
    
    scenario.updatePlots();
    drawnow limitrate;
    
    % FIX: Real-time pacing (1x speed instead of 10x slow-mo)
    elapsed = toc(tLoop);
    targetT = stepN * scenario.SampleTime; 
    if elapsed < targetT; pause(targetT - elapsed); end
    
    if egoPos(1) >= 243
        metrics.GoalReached = true;
        fprintf('[S5] GOAL! x=%.0f YOLO-cows=%d\n', egoPos(1), mlLog.yoloCows);
        break;
    end
end
PrintSummary(metrics);
fprintf('[S5] Scenario 5 Complete.\n');
end

function c = sc(s)
    if contains(s,'LATERAL'); c=[0.3 0.8 1]; return; end
    switch s
        case 'CRUISE';          c=[0.2 0.9 0.4];
        case 'CAUTION';         c=[1.0 0.8 0.1];
        case 'BRAKING';         c=[1.0 0.4 0.1];
        case 'STOPPED';         c=[1.0 0.1 0.1];
        case 'CATTLE-ZONE';     c=[1.0 0.5 0.1];
        case 'YOLO-COW';        c=[1.0 0.3 0.0];
        case 'YOLO-SHEPHERD';   c=[0.8 0.4 1.0];
        otherwise;              c=[1.0 1.0 1.0];
    end
end