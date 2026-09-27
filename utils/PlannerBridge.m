function [safeVel, shiftedWP, status, minDist, latState] = PlannerBridge(wayPoints, egoPos, egoYaw, actorPoses, egoID, nominalSpeed, latState)
% PlannerBridge - Unified path planner drop-in replacement
%
% Replaces the two separate calls to CollisionAvoidance and LateralAvoidance
% with a single function that integrates:
%   1. CollisionAvoidance  — longitudinal speed control (unchanged)
%   2. CSpacePlanner       — multi-candidate C-Space lateral planning (NEW)
%   3. VelocityProfiler    — curvature-based speed limiting (NEW)
%
% The key improvement over LateralAvoidance.m:
%   - CSpacePlanner evaluates 21 lateral candidates and picks the SAFEST side
%   - It no longer always shifts LEFT — it correctly detects which side is free
%   - Falls back to LateralAvoidance if CSpacePlanner finds a BLOCKED route
%
% Usage (replaces lines in every scenario loop):
%   % Before loop: latState = [];  prevVel = nominalSpeed;  pStep = 0;
%
%   [safeVel, shiftedWP, status, minDist, latState] = ...
%       PlannerBridge(wayPoints, egoPos, egoYaw, aPoses, egoID, nominalSpeed, latState);
%   ReplanTrajectory(ego, shiftedWP, egoPos, safeVel, prevVel, pStep);
%   prevVel = safeVel;  pStep = pStep + 1;
%
% Inputs:
%   wayPoints    - Nx3 original waypoint matrix
%   egoPos       - [x y z] current ego position
%   egoYaw       - ego heading (radians)
%   actorPoses   - struct array with .Position and .ActorID for each actor
%   egoID        - ego actor ID (excluded from obstacle set)
%   nominalSpeed - desired cruise speed (m/s)
%   latState     - persistent state struct (pass [] on first call)
%
% Outputs:
%   safeVel    - final commanded speed (m/s) after all constraints
%   shiftedWP  - Nx3 (possibly laterally shifted) waypoints
%   status     - string: 'CRUISE'|'CAUTION'|'BRAKING'|'STOPPED'|'LATERAL-LEFT'|
%                        'LATERAL-RIGHT'|'LATERAL-BLOCKED'
%   minDist    - minimum distance to nearest threat (m)
%   latState   - updated state struct for next call

    %% Initialise persistent lateral state
    if isempty(latState)
        latState.shiftAmount  = 0;
        latState.targetShift  = 0;
        latState.clearCount   = 0;
        latState.mode         = 'CLEAR';
        latState.cspaceCfg    = cspace_default_config();
        latState.planFrame    = 0;      % frame counter for plan caching
        latState.cachedSide   = 'CENTER';
        latState.cachedShift  = 0;
        latState.cachedSafe   = true;
    end

    SHIFT_SPEED     = 0.20;   % shift ramp rate (m/frame) - increased for faster dodging
    CLEAR_THRESHOLD = 15;     % frames clear before returning to centre

    %% 1. Filter out ego from actor list
    % (Actors are already filtered in the scenario loop)
    aPoses = actorPoses;
    %% 0. Make waypoints dense (1m spacing) so kinematic follower steers sharply
    denseWP = [];
    for i = 1:size(wayPoints,1)-1
        p1 = wayPoints(i,:); p2 = wayPoints(i+1,:);
        dist = norm(p2-p1);
        numPts = max(2, ceil(dist / 1.0));
        x_interp = linspace(p1(1), p2(1), numPts)';
        y_interp = linspace(p1(2), p2(2), numPts)';
        z_interp = linspace(p1(3), p2(3), numPts)';
        denseWP = [denseWP; [x_interp(1:end-1), y_interp(1:end-1), z_interp(1:end-1)]];
    end
    denseWP = [denseWP; wayPoints(end,:)];
    wayPoints = denseWP;

    %% 1. Predict Collision & Speed
    [collisionVel, status, minDist] = CollisionAvoidance( ...
        [egoPos(1), egoPos(2), egoYaw], aPoses, nominalSpeed);

    %% 3. Lateral: CSpacePlanner — multi-candidate beam search
    % Calculate road heading to pass to CSpacePlanner
    % If we use egoYaw, the planner evaluates straight paths in the direction the Ego is currently facing
    % which is WRONG if the Ego is swerving (e.g. 68 degrees to dodge an obstacle).
    % We must evaluate paths parallel to the ROAD!
    roadVec  = wayPoints(end, 1:2) - wayPoints(1, 1:2);
    roadYaw  = atan2(roadVec(2), roadVec(1));
    
    %   Only run if not at emergency stop (save compute cycles at speed=0)
    cspaceSide    = latState.cachedSide;
    cspaceShift   = latState.cachedShift;
    cspaceBlocked = ~latState.cachedSafe;

    latState.planFrame = latState.planFrame + 1;
    % Re-run CSpacePlanner every 5 frames only (cache result in between)
    if mod(latState.planFrame, 5) == 1 && ~strcmp(status, 'STOPPED') && ~isempty(aPoses)
        try
            % Pass roadYaw instead of egoYaw!
            % Pass current shift amount so planner evaluates absolute road paths
            [cs, cd, csSafe, ~] = CSpacePlanner(egoPos, roadYaw, aPoses, latState.cspaceCfg, latState.shiftAmount);
            latState.cachedShift = cs;
            latState.cachedSide  = cd;
            latState.cachedSafe  = csSafe;
            cspaceShift   = cs;
            cspaceSide    = cd;
            cspaceBlocked = ~csSafe;
        catch
            % keep cached values
        end
    end

    %% 4. Lateral shift tracking with Hysteresis
    if ~cspaceBlocked
        % If planner wants to return to center (0), but we are currently shifted, delay it!
        if abs(cspaceShift) < 0.1 && abs(latState.shiftAmount) > 0.5
            latState.clearCount = latState.clearCount + 1;
            if latState.clearCount > CLEAR_THRESHOLD
                latState.targetShift = 0;
            end
        else
            latState.clearCount = 0;
            latState.targetShift = cspaceShift;
        end
    else
        % If blocked, hold the current target shift
        latState.clearCount = 0;
    end

    % Ramp shift toward target
    err = latState.targetShift - latState.shiftAmount;
    step = sign(err) * min(abs(err), SHIFT_SPEED);
    latState.shiftAmount = latState.shiftAmount + step;
    
    % ── ROAD BOUNDARY CLAMP ────────────────────────────────────────────
    % xHalf = 3.5m, so max valid lateral shift = 3.0m (0.5m safety margin).
    MAX_SHIFT = 3.0;
    latState.shiftAmount = max(-MAX_SHIFT, min(MAX_SHIFT, latState.shiftAmount));
    latState.targetShift  = max(-MAX_SHIFT, min(MAX_SHIFT, latState.targetShift));
    % ───────────────────────────────────────────────────────────────────
    
    if abs(latState.targetShift) > 0.05
        fprintf('  [CSPACE] Target Shift: %.2fm (Current: %.2fm)\n', latState.targetShift, latState.shiftAmount);
    end

    %% 5. Apply lateral shift to waypoints
    shift    = latState.shiftAmount;
    
    % FIX: Use the overall road direction to determine lateral shift direction!
    % If we use egoYaw, the car turning causes the waypoints to spin into a spiral!
    roadVec  = wayPoints(end, 1:2) - wayPoints(1, 1:2);
    roadYaw  = atan2(roadVec(2), roadVec(1));
    perpX    = -sin(roadYaw);   % perpendicular direction (world frame)
    perpY    =  cos(roadYaw);

    shiftedWP = wayPoints;

    % ── ROAD BOUNDARY CLAMP ON WAYPOINTS (curved-road aware) ────────────────
    % For a curved road the centreline Y changes with X.
    % Each shifted waypoint must stay within ±HALF_WIDTH of the ORIGINAL
    % waypoint Y at that X (i.e. the road centreline at that point).
    WP_HALF_WIDTH = 3.0;   % metres each side of the original centreline
    % ────────────────────────────────────────────────────────────────────────

    if abs(shift) >= 0.05
        for i = 1:size(wayPoints, 1)
            ahead_dist = (wayPoints(i,1)-egoPos(1))*cos(egoYaw) + ...
                         (wayPoints(i,2)-egoPos(2))*sin(egoYaw);
            if ahead_dist > 0
                origY = wayPoints(i,2);   % road centreline Y at this waypoint
                newY  = origY + perpY * shift;
                newX  = wayPoints(i,1) + perpX * shift;
                % Clamp relative to this waypoint's OWN centreline
                newY  = max(origY - WP_HALF_WIDTH, min(origY + WP_HALF_WIDTH, newY));
                shiftedWP(i,1) = newX;
                shiftedWP(i,2) = newY;
            end
        end
    end

    %% 6. Update status and override STOP if we have a safe bypass path
    if ~cspaceBlocked && abs(shift) > 0.05
        % We are safely bypassing the obstacle laterally!
        % Prevent longitudinal controller from forcing a dead stop
        if collisionVel < 3.0
            collisionVel = 3.0; % keep moving while passing
        end
        if shift > 0
            status = 'LATERAL-LEFT';
        else
            status = 'LATERAL-RIGHT';
        end
    elseif cspaceBlocked && ~strcmp(status, 'STOPPED')
        status = 'LATERAL-BLOCKED';
    end

    %% 7. Velocity: curvature-based speed limit on shifted waypoints
    safeVel = VelocityProfiler(shiftedWP, egoPos, nominalSpeed, collisionVel);
end


%% =========================================================================
function cfg = cspace_default_config()
    cfg.resolution        = 0.25;   % 25 cm cells — 25x faster than 5cm
    cfg.zMax              = 60.0;   % look-ahead distance (m)
    cfg.xHalf             = 3.5;    % ±3.5 m = ONE full road half-width.
    %                                  Candidates only span the actual road,
    %                                  so the planner CANNOT pick off-road paths.
    cfg.egoHalfWidth      = 0.85;
    cfg.actorHalfWidth    = 0.60;
    cfg.actorHalfLength   = 1.50;
    cfg.uncertaintyBuffer = 0.30;
    cfg.additionalBuffer  = 0.10;
    cfg.requiredClearance = 0.40;
    cfg.nCandidates       = 11;
    cfg.planningHorizon   = 55.0;
    cfg.sideThreshold     = 0.10;
end
