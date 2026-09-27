function metrics = Scenario4_MarketArea(mlModels)
% SCENARIO 4: Dense Market Area — Fixed motion + rich moving obstacles
% Ego navigates a busy Indian market street at very low speed.
if nargin < 1; mlModels = struct('yolo',[],'deeplab',[],'cnn',[]); end
addpath(fullfile(fileparts(mfilename('fullpath')),'..','utils'));
fprintf('\n[S4] Starting Scenario 4: Dense Market Area...\n');
fprintf('[S4] Watch: pushcart, pedestrians, parked auto, two-wheelers, vendor\n');

%% ── Build scenario ──────────────────────────────────────────────────────
scenario = drivingScenario('SampleTime',0.05,'StopTime',150);

road(scenario, [0 0 0;20 0 0;40 1 0;60 2 0;80 2 0;100 1 0;120 0 0],...
     'Lanes', lanespec(1,'Width',3.5));

%% ── Ego vehicle ─────────────────────────────────────────────────────────
ego = vehicle(scenario,'ClassID',1,'Length',4.0,'Width',1.7,'Height',1.5,...
              'PlotColor',[0 0.85 0.3]);
egoID = ego.ActorID;
wayPoints = [0 0 0; 20 0 0; 40 1 0; 60 2 0; 80 2 0; 100 1 0; 118 0 0];
nominalSpeed = 3.0;   % very slow market speed
% --- Custom Kinematic Initialization ---
ego.Position = wayPoints(1, 1:3);
ego.Velocity = [0 0 0];
ego.Yaw = 0;

currentWaypointIndex = 2;
isHalted = false;
latState = [];
stuckTimer = 0;            % counts frames ego is halted
STUCK_MAX_FRAMES = 120;   % 6 s at 0.05 s/frame — then creep at 1 m/s


%% ── Moving obstacles ────────────────────────────────────────────────────
% 1. Slow pushcart moving in same direction ahead
pushcart = actor(scenario,'ClassID',5,'Length',1.5,'Width',0.8,'Height',1.2,...
                 'PlotColor',[0.9 0.6 0.1]);
trajectory(pushcart, [8 0.5 0; 70 0.5 0], [0; 62]);   % 62m / 1m/s = 62s
% 2. Parked auto-rickshaw — actor() because ClassID=7 not valid for vehicle()
parkedAuto = actor(scenario,'ClassID',7,'Length',2.8,'Width',1.3,'Height',1.7,...
                     'PlotColor',[1 0.5 0]);
trajectory(parkedAuto, [35 2.5 0; 35.01 2.5 0], [0; 60]);  % stationary
% 3. Pedestrian 1 — starts off-road, approaches then crosses at t=8
ped1 = actor(scenario,'ClassID',4,'Length',0.5,'Width',0.5,'Height',1.7,...
             'PlotColor',[1 0.3 0.3]);
trajectory(ped1, [50 -10 0; 50 -3.5 0; 50 5 0], [0; 8; 15]);
% 4. Pedestrian 2 — starts off-road, approaches then crosses at t=12
ped2 = actor(scenario,'ClassID',4,'Length',0.5,'Width',0.5,'Height',1.7,...
             'PlotColor',[1 0.7 0.2]);
trajectory(ped2, [80 12 0; 80 5 0; 80 -3.5 0], [0; 12; 19]);
% 5. Two-wheeler — actor() because ClassID=3 not valid for vehicle()
bike = actor(scenario,'ClassID',3,'Length',2.0,'Width',0.7,'Height',1.2,...
               'PlotColor',[0.8 0.2 1]);
trajectory(bike, [5 0 0; 30 1.5 0; 55 0 0; 90 1 0; 115 0.5 0],...
                 [0;   8;    16;    24;    32]);
% 6. Child — starts off-road, approaches then crosses at t=14
child = actor(scenario,'ClassID',4,'Length',0.4,'Width',0.4,'Height',1.0,...
              'PlotColor',[0.5 1 0.5]);
trajectory(child, [68 10 0; 68 4.5 0; 68 -2 0], [0; 14; 20]);
% 7. Oncoming auto-rickshaw — actor() because ClassID=7 not valid for vehicle()
auto2 = actor(scenario,'ClassID',7,'Length',2.8,'Width',1.3,'Height',1.7,...
                'PlotColor',[0.2 0.6 1]);
trajectory(auto2, [115 1.0 0; 55 1.0 0; 5 0.5 0], [0; 18; 36]);
% 8. Pedestrian 3 — starts off-road, approaches then crosses at t=22
ped3 = actor(scenario,'ClassID',4,'Length',0.5,'Width',0.5,'Height',1.7,...
             'PlotColor',[1 1 0.3]);
trajectory(ped3, [98 -8 0; 98 -1.5 0; 98 3 0], [0; 22; 28]);
% 9. Stationary cow (sitting in road)
cow = actor(scenario,'ClassID',5,'Length',2.0,'Width',0.8,'Height',1.2,...
            'PlotColor',[0.6 0.4 0.2]);
trajectory(cow, [62 1.5 0; 62.01 1.5 0], [0; 60]);   % stationary cow

%% ── Figures ─────────────────────────────────────────────────────────────
fig1 = figure('Name','S4:Market-Chase','NumberTitle','off',...
              'Color',[0.12 0.07 0.07],'Position',[30 380 700 400]);
ax1  = axes(fig1,'Color',[0.12 0.07 0.07]);
chasePlot(ego,'Parent',ax1,'Centerline','on');
title(ax1,'[S4] Market Area — Chase View (GREEN = EGO, very slow)',...
      'Color','w','FontSize',10,'FontWeight','bold');

fig2 = figure('Name','S4:Market-Top','NumberTitle','off',...
              'Color',[0.12 0.07 0.07],'Position',[740 380 700 400]);
ax2  = axes(fig2,'Color',[0.16 0.1 0.1],'XColor','w','YColor','w');
xlabel(ax2,'X (m)','Color','w'); ylabel(ax2,'Y (m)','Color','w');
hold(ax2,'on');

% Parked auto zone
patch(ax2,[30 42 42 30],[-1 -1 4.5 4.5],[1 0.5 0],...
      'FaceAlpha',0.15,'EdgeColor',[1 0.5 0],'LineWidth',1);
text(ax2,36,5.5,'PARKED AUTO','Color',[1 0.6 0.2],...
     'FontSize',7,'HorizontalAlignment','center');
% Cow zone
patch(ax2,[58 66 66 58],[-0.5 -0.5 3.5 3.5],[0.6 0.4 0.2],...
      'FaceAlpha',0.15,'EdgeColor',[0.6 0.4 0.2],'LineWidth',1);
text(ax2,62,-1.5,'COW','Color',[0.7 0.5 0.3],...
     'FontSize',7,'HorizontalAlignment','center');

plot(ax2, wayPoints(:,1), wayPoints(:,2),'w--','LineWidth',1.5,'DisplayName','Ego path');

% Moving markers
egoMk   = plot(ax2, 0,   0,   'gs','MarkerSize',12,'MarkerFaceColor','g','DisplayName','EGO');
cartMk  = plot(ax2, 8,   0.5, 'ys','MarkerSize', 9,'MarkerFaceColor',[0.9 0.6 0.1],'DisplayName','Pushcart');
bikeMk  = plot(ax2, 5,   0,   'ms','MarkerSize', 8,'MarkerFaceColor','m','DisplayName','Bike');
auto2Mk = plot(ax2,115,  1.0, 'bs','MarkerSize', 9,'MarkerFaceColor','b','DisplayName','Auto2');

legend(ax2,'show','TextColor','w','Color',[0.16 0.1 0.1],'FontSize',7,'Location','northeast');
title(ax2,'[S4] Market Top View (LIVE) — full 120m visible',...
      'Color','w','FontSize',9,'FontWeight','bold');
xlim(ax2,[-3 125]); ylim(ax2,[-7 9]); grid(ax2,'on'); axis(ax2,'manual');

%% ── Simulation loop ──────────────────────────────────────────────────────
metrics    = MetricsLogger('Scenario4_MarketArea');
FRAME_SIZE = [120 160 3]; roadFrac = 0.5;
mlLog.yoloPeds = 0; stepN = 0; trajX = []; trajY = [];

% FIX: Pre-allocate trajectory line for real-time update
trajLine = plot(ax2, NaN, NaN, 'c-', 'LineWidth', 2.5, 'Tag', 'tr');

fprintf('[S4] Simulation running...\n');
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
    cP=[8,0.5]; bP=[5,0]; a2P=[115,1.0];
    
    for k = 1:numel(ap)
        aID = scenario.Actors(k).ActorID;
        if aID ~= egoID
            idx = idx+1;
            aPoses(idx).Position = ap(k).Position;
            aPoses(idx).Width = scenario.Actors(k).Width;
            aPoses(idx).Length = scenario.Actors(k).Length;
            if aID == pushcart.ActorID; cP  = ap(k).Position(1:2); end
            if aID == bike.ActorID;     bP  = ap(k).Position(1:2); end
            if aID == auto2.ActorID;    a2P = ap(k).Position(1:2); end
        end
    end
    
    % --- DYNAMIC TRAJECTORY REPLANNING (PlannerBridge) ---
    if exist('wayPoints', 'var'); wps = wayPoints; elseif exist('wpNormal', 'var'); wps = wpNormal; else; wps = []; end
    if ~exist('latState', 'var'); latState = []; end
    
    [safeVel, shiftedWP, status, minDist, latState] = PlannerBridge(wps, egoPos, egoYaw, aPoses, egoID, nominalSpeed, latState);
    
    inPark = egoPos(1) >= 28 && egoPos(1) <= 44;
    if inPark && strcmp(status,'CRUISE'); status = 'PASSING-PARKED'; end
    
    frame = SyntheticCamera(scenario, egoPos, FRAME_SIZE(1:2));
    [~,~,~,det] = RunYOLO(frame, mlModels.yolo, 0.80);
    if det.hasPerson && det.closestDist < 8
        status = 'YOLO-PED-STOP'; mlLog.yoloPeds = mlLog.yoloPeds + 1;
    elseif det.hasPerson && det.closestDist < 15
        status = 'YOLO-PED-SLOW'; mlLog.yoloPeds = mlLog.yoloPeds + 1;
    end
    roadFrac = RunDeepLab(frame, mlModels.deeplab, [360 480 3]);
    
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
        stuckTimer = stuckTimer + 1;
    else
        isHalted = false;
        stuckTimer = 0;
    end
    % STUCK TIMEOUT: if blocked > 6 s in dense market, creep through at 2.5 m/s
    if stuckTimer > STUCK_MAX_FRAMES
        isHalted = false;
        safeVel   = 2.5;
        status    = 'CREEP-MARKET';
    end
    
    % --- 3. Kinematic Execution ---
    if isHalted || (~isempty(shiftedWP) && currentWaypointIndex > size(shiftedWP, 1))
        ego.Velocity = [0, 0, 0];
    else
        ego.Velocity = [safeVel * cosd(ego.Yaw), safeVel * sind(ego.Yaw), 0];
    end
    
    ego.Position = ego.Position + ego.Velocity * scenario.SampleTime;
    % ── ROAD BOUNDARY CLAMP (Dynamic — follows curved road) ───────────────
    ROAD_HALF_WIDTH = 2.5;
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
        title(ax1, sprintf('[S4] x=%.1fm  spd=%.2fm/s | %s | Peds:%d Road:%.0f%%',...
            egoPos(1), currSpeed, status, mlLog.yoloPeds, roadFrac*100),...
            'Color',c,'FontSize',9,'FontWeight','bold');
    catch; end
    
    try
        % FIX: Update XData/YData instead of destroying and recreating the plot
        set(trajLine, 'XData', trajX, 'YData', trajY);
        set(egoMk,   'XData',egoPos(1), 'YData',egoPos(2));
        set(cartMk,  'XData',cP(1),     'YData',cP(2));
        set(bikeMk,  'XData',bP(1),     'YData',bP(2));
        set(auto2Mk, 'XData',a2P(1),    'YData',a2P(2));
    catch; end
    
    scenario.updatePlots();
    drawnow limitrate;
    
    % FIX: Real-time pacing (1x speed instead of 10x)
    elapsed = toc(tLoop);
    targetT = stepN * scenario.SampleTime; 
    if elapsed < targetT; pause(targetT - elapsed); end
    
    if egoPos(1) >= 116
        metrics.GoalReached = true;
        fprintf('[S4] GOAL! x=%.1f Peds=%d\n', egoPos(1), mlLog.yoloPeds);
        break;
    end
end
PrintSummary(metrics);
fprintf('[S4] Scenario 4 Complete.\n');
end

function c = sc(s)
    if contains(s,'LATERAL'); c=[0.3 0.8 1]; return; end
    switch s
        case 'CRUISE';          c=[0.2 0.9 0.4];
        case 'CAUTION';         c=[1.0 0.8 0.1];
        case 'BRAKING';         c=[1.0 0.4 0.1];
        case 'STOPPED';         c=[1.0 0.1 0.1];
        case 'PASSING-PARKED';  c=[0.8 0.5 1.0];
        case 'YOLO-PED-STOP';   c=[1.0 0.0 0.0];
        case 'YOLO-PED-SLOW';   c=[1.0 0.4 0.1];
        case 'DEEPLAB-NARROW';  c=[0.4 0.7 1.0];
        otherwise;              c=[1.0 1.0 1.0];
    end
end