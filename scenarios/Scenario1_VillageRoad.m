function metrics = Scenario1_VillageRoad(mlModels)
% SCENARIO 1: Unmarked Village Road — Fixed motion + rich moving obstacles
% All obstacles move visibly. Ego follows road in slow-motion for 10s demo.
if nargin < 1; mlModels = struct('yolo',[],'deeplab',[],'cnn',[]); end
addpath(fullfile(fileparts(mfilename('fullpath')),'..','utils'));
fprintf('\n[S1] Starting Scenario 1: Unmarked Village Road...\n');
fprintf('[S1] Watch for: pedestrians crossing, oncoming bike, bullock cart, dog\n');

%% ── Build scenario ──────────────────────────────────────────────────────
scenario = drivingScenario('SampleTime',0.05,'StopTime',45);
roadCenters = [0 0 0; 40 0 0; 80 4 0; 120 8 0; 160 10 0; 200 8 0];
road(scenario, roadCenters, 'Lanes', lanespec(1,'Width',4.0));

%% ── Ego vehicle ─────────────────────────────────────────────────────────
ego = vehicle(scenario,'ClassID',1,'Length',4.7,'Width',1.4,'Height',1.5,...
              'PlotColor',[0 0.8 0.3]);
egoID = ego.ActorID;
wayPoints = [0 0 0; 40 0 0; 80 4 0; 120 8 0; 160 10 0; 200 8 0];
nominalSpeed = 6;   % m/s ≈ 22 km/h village speed
% --- Custom Kinematic Initialization ---
ego.Position = wayPoints(1, 1:3);
ego.Velocity = [0 0 0];
ego.Yaw = 0;

currentWaypointIndex = 2;
isHalted = false;
latState = [];

%% ── Moving obstacles ────────────────────────────────────────────────────
% 1. Pedestrian crossing left→right at x=55
ped1 = actor(scenario,'ClassID',4,'Length',0.5,'Width',0.5,'Height',1.7,...
             'PlotColor',[1 0.4 0]);
trajectory(ped1, [55 -6 0; 55 8 0], [0; 11.7]);   % 14m / 1.2m/s ≈ 11.7s
% 2. Second pedestrian — starts off-road at t=0, walks to road edge at t=8, crosses by t=20
ped2 = actor(scenario,'ClassID',4,'Length',0.5,'Width',0.5,'Height',1.7,...
             'PlotColor',[1 0.7 0]);
trajectory(ped2, [95 18 0; 95 10 0; 95 -4 0], [0; 8; 20]);  % approaches then crosses
% 3. Oncoming motorcycle — actor() because ClassID=3 not valid for vehicle()
bike = actor(scenario,'ClassID',3,'Length',2.0,'Width',0.7,'Height',1.2,...
               'PlotColor',[0.9 0.2 0.8]);
trajectory(bike, [195 9 0; 80 3 0], [0; 19]);   % comes towards ego
% 4. Bullock cart (slow, same direction) starting 40m ahead
cart = vehicle(scenario,'ClassID',2,'Length',3.5,'Width',1.5,'Height',1.8,...
               'PlotColor',[0.7 0.5 0.2]);
trajectory(cart, [40 0 0; 190 8 0], [0; 64]);     % 160m / 2.5m/s = 64s
% 5. Dog darting across road — starts off-road, approaches then darts across
dog = actor(scenario,'ClassID',5,'Length',0.8,'Width',0.3,'Height',0.4,...
            'PlotColor',[0.8 0.6 0.1]);
trajectory(dog, [130 25 0; 130 12 0; 130 6 0; 135 2 0], [0; 12; 15; 18]);
% 6. Parked auto-rickshaw (stationary obstacle) on side
autoRick = vehicle(scenario,'ClassID',7,'Length',2.8,'Width',1.3,'Height',1.7,...
                   'PlotColor',[1 0.6 0]);
trajectory(autoRick, [110 11 0; 110.01 11 0], [0; 10]);  % essentially stationary
% 7. Child running across — starts off-road, then runs across at t=9
child = actor(scenario,'ClassID',4,'Length',0.4,'Width',0.4,'Height',1.0,...
              'PlotColor',[0.5 1 0.5]);
trajectory(child, [75 -15 0; 75 -5 0; 75 7 0], [0; 9; 14]);

%% ── Figures ─────────────────────────────────────────────────────────────
fig1 = figure('Name','S1:Village-Chase','NumberTitle','off',...
              'Color',[0.06 0.08 0.12],'Position',[30 380 700 400]);
ax1  = axes(fig1,'Color',[0.06 0.08 0.12]);
chasePlot(ego,'Parent',ax1,'Centerline','on');
title(ax1,'[S1] Village Road — Chase View (ego = GREEN car)',...
      'Color','w','FontSize',10,'FontWeight','bold');

fig2 = figure('Name','S1:Village-Top','NumberTitle','off',...
              'Color',[0.06 0.08 0.12],'Position',[740 380 700 400]);
ax2  = axes(fig2,'Color',[0.1 0.1 0.16],'XColor','w','YColor','w');
xlabel(ax2,'X (m)','Color','w'); ylabel(ax2,'Y (m)','Color','w');
hold(ax2,'on');

% Draw road outline
plot(ax2, roadCenters(:,1), roadCenters(:,2), 'w--','LineWidth',1,'DisplayName','Road Centre');
% Obstacle markers
plot(ax2, 55,  0, 'r^','MarkerSize',10,'MarkerFaceColor','r','DisplayName','Ped crossing');
plot(ax2, 95,  8, 'r^','MarkerSize',10,'MarkerFaceColor',[1 0.7 0],'DisplayName','Ped 2');
plot(ax2,130,  9, 'kd','MarkerSize', 8,'MarkerFaceColor','k','DisplayName','Dog');
plot(ax2,110, 11, 'ys','MarkerSize', 8,'MarkerFaceColor','y','DisplayName','Parked auto');
egoMk  = plot(ax2, 0, 0,'gs','MarkerSize',12,'MarkerFaceColor','g','DisplayName','EGO');
bikeMk = plot(ax2, 195, 9,'ms','MarkerSize',8,'MarkerFaceColor','m','DisplayName','Bike');
cartMk = plot(ax2, 40,  0,'bs','MarkerSize',8,'MarkerFaceColor',[0.7 0.5 0.2],'DisplayName','Cart');

legend(ax2,'show','TextColor','w','Color',[0.1 0.1 0.16],'FontSize',7,'Location','northwest');
title(ax2,'[S1] Village Road — Top View (LIVE)','Color','w','FontSize',10,'FontWeight','bold');
xlim(ax2,[-5 205]); ylim(ax2,[-10 20]); grid(ax2,'on'); axis(ax2,'manual');

%% ── Simulation loop ──────────────────────────────────────────────────────
metrics  = MetricsLogger('Scenario1_VillageRoad');
FRAME_SIZE = [120 160 3]; roadFrac = 0.5;
mlLog.yoloHits = 0; stepN = 0; trajX = []; trajY = [];

% FIX: Initialize trajectory line BEFORE the loop instead of recreating it inside
trajLine = plot(ax2, NaN, NaN, 'c-', 'LineWidth', 2.5, 'Tag', 'tr');

fprintf('[S1] Simulation running — watch figures for motion...\n');
tLoop = tic;
while advance(scenario)
    stepN = stepN + 1;
    % Ego state (Manual Read)
    egoPos   = ego.Position;
    egoVel   = ego.Velocity;
    currSpeed= norm(egoVel(1:2));
    if currSpeed > 0.01
        egoYaw = atan2(egoVel(2), egoVel(1)+1e-9);
    else
        egoYaw = ego.Yaw * pi/180;
    end
    
    % All actor poses (for collision avoidance HUD display only)
    ap = actorPoses(scenario);
    aPoses = struct('Position',{}, 'Width',{}, 'Length',{}); idx = 0;
    bikePos = [195 9]; cartPos = [40 0];
    
    for k = 1:numel(ap)
        aID = scenario.Actors(k).ActorID;
        if aID ~= egoID
            idx = idx+1;
            aPoses(idx).Position = ap(k).Position;
            aPoses(idx).Width = scenario.Actors(k).Width;
            aPoses(idx).Length = scenario.Actors(k).Length;
            if aID == bike.ActorID; bikePos = ap(k).Position(1:2); end
            if aID == cart.ActorID; cartPos = ap(k).Position(1:2); end
        end
    end
    
    % --- DYNAMIC TRAJECTORY REPLANNING (PlannerBridge) ---
    if exist('wayPoints', 'var'); wps = wayPoints; elseif exist('wpNormal', 'var'); wps = wpNormal; else; wps = []; end
    if ~exist('latState', 'var'); latState = []; end
    
    [safeVel, shiftedWP, status, minDist, latState] = PlannerBridge(wps, egoPos, egoYaw, aPoses, egoID, nominalSpeed, latState);
    
    % ML stubs
    frame = SyntheticCamera(scenario, egoPos, FRAME_SIZE(1:2));
    [~,~,~,det] = RunYOLO(frame, mlModels.yolo, 0.80);
    if det.hasPerson && det.closestDist < 12; status = 'YOLO-PERSON'; mlLog.yoloHits = mlLog.yoloHits+1; end
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
    % ── ROAD BOUNDARY CLAMP (Dynamic — follows curved road) ─────────────
    % Interpolate road centreline Y at ego's current X from wayPoints table.
    % Then clamp ego Y within ±HALF_WIDTH of that centreline.
    ROAD_HALF_WIDTH = 3.5;   % half the full road width (2 lanes × 3.5m)
    roadX = wayPoints(:,1);
    roadY = wayPoints(:,2);
    roadCentreY = interp1(roadX, roadY, ego.Position(1), 'linear', 'extrap');
    ego.Position(2) = max(roadCentreY - ROAD_HALF_WIDTH, ...
                      min(roadCentreY + ROAD_HALF_WIDTH, ego.Position(2)));
    % If pressing against a boundary, straighten yaw (stop steering into wall)
    if ego.Position(2) <= roadCentreY - ROAD_HALF_WIDTH + 0.3 && ego.Yaw < -5
        ego.Yaw = max(ego.Yaw, -5);
    end
    if ego.Position(2) >= roadCentreY + ROAD_HALF_WIDTH - 0.3 && ego.Yaw > 5
        ego.Yaw = min(ego.Yaw, 5);
    end
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
        lateralBuffer = 0.1; % Small padding for lateral passing
        
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
    
    % Update HUD title
    c = sc(status);
    try
        title(ax1, sprintf('[S1] x=%.1fm  spd=%.1fm/s | %s | Road:%.0f%% YOLO:%d',...
            egoPos(1), currSpeed, status, roadFrac*100, mlLog.yoloHits),...
            'Color',c,'FontSize',9,'FontWeight','bold');
    catch; end
    
    % Update top-view
    try
        % FIX: Directly update XData and YData properties of the pre-initialized line
        set(trajLine, 'XData', trajX, 'YData', trajY);
        set(egoMk,  'XData',egoPos(1),   'YData',egoPos(2));
        set(bikeMk, 'XData',bikePos(1),  'YData',bikePos(2));
        set(cartMk, 'XData',cartPos(1),  'YData',cartPos(2));
    catch; end
    
    scenario.updatePlots();
    drawnow limitrate;
    
    % FIX: Pacing updated for real-time smoothness (removed the * 10 multiplier)
    elapsed  = toc(tLoop);
    targetT  = stepN * scenario.SampleTime;   % Now 1x speed instead of 10x slow-mo
    if elapsed < targetT
        pause(targetT - elapsed);
    end
    
    if egoPos(1) >= 198
        metrics.GoalReached = true;
        fprintf('[S1] GOAL reached! x=%.1f  YOLO hits=%d\n', egoPos(1), mlLog.yoloHits);
        break;
    end
end
PrintSummary(metrics);
fprintf('[S1] Scenario 1 Complete.\n');
end

function c = sc(s)
    if contains(s,'LATERAL'); c=[0.3 0.8 1.0]; return; end
    switch s
        case 'CRUISE';      c=[0.2 0.9 0.4];
        case 'CAUTION';     c=[1.0 0.8 0.1];
        case 'BRAKING';     c=[1.0 0.4 0.1];
        case 'STOPPED';     c=[1.0 0.1 0.1];
        case 'YOLO-PERSON'; c=[0.8 0.3 1.0];
        otherwise;          c=[1.0 1.0 1.0];
    end
end