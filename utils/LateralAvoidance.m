function [shiftedWaypoints, lateralState] = LateralAvoidance(wayPoints, egoPos, egoYaw, ...
                                                               actorPoses, egoID, lateralState)
% LateralAvoidance — Shift ego path laterally when obstacle detected in lane
%
% When an obstacle is directly ahead in the ego lane, this function
% generates a SHIFTED set of waypoints that curve around the obstacle,
% then return to the original path. This is the "path runner" behaviour.
%
% Inputs:
%   wayPoints   - Nx3 original waypoint matrix
%   egoPos      - [x y z] current ego position
%   egoYaw      - current ego heading (radians)
%   actorPoses  - struct array with .Position for each other actor
%   egoID       - ego actor ID (to exclude from checks)
%   lateralState- persistent state struct (pass [] on first call)
%
% Outputs:
%   shiftedWaypoints - Nx3 (possibly laterally shifted) waypoints
%   lateralState     - updated state struct for next call
%
% State machine:
%   CLEAR       → normal path (no shift)
%   SHIFTING    → actively detouring around obstacle
%   RETURNING   → coming back to original lane

    %% Init state
    if isempty(lateralState)
        lateralState.mode          = 'CLEAR';
        lateralState.shiftAmount   = 0;       % metres (+ = left, - = right)
        lateralState.targetShift   = 0;
        lateralState.clearCount    = 0;
        lateralState.obstacleX     = Inf;
    end

    %% Parameters
    DETECT_RANGE    = 18;   % m — look this far ahead for obstacles in-lane
    LANE_WIDTH      = 3.5;  % m — half-width of own lane for in-lane check
    LATERAL_OFFSET  = 2.8;  % m — how far to shift when avoiding
    SHIFT_SPEED     = 0.15; % shift amount per frame (smooth)
    CLEAR_THRESHOLD = 8;    % frames clear before returning

    %% Check for in-lane obstacle ahead
    inLaneObstacle = false;
    obstaclePos    = [];

    for k = 1:numel(actorPoses)
        ap  = actorPoses(k).Position;
        dx  = ap(1) - egoPos(1);
        dy  = ap(2) - egoPos(2);

        % Transform to ego-local frame
        localX =  dx*cos(egoYaw) + dy*sin(egoYaw);
        localY = -dx*sin(egoYaw) + dy*cos(egoYaw);

        % In-lane = ahead within DETECT_RANGE AND within lane width
        if localX > 0.5 && localX < DETECT_RANGE && abs(localY) < LANE_WIDTH
            inLaneObstacle = true;
            obstaclePos    = ap;
            lateralState.obstacleX = ap(1);
            break;
        end
    end

    %% Decide shift direction (go to whichever side has more space)
    if inLaneObstacle && isempty(lateralState.shiftAmount == 0)
        lateralState.targetShift = LATERAL_OFFSET;  % default: shift left
    end

    %% State machine
    switch lateralState.mode
        case 'CLEAR'
            if inLaneObstacle
                % Choose shift direction: prefer left (positive y)
                lateralState.targetShift = LATERAL_OFFSET;
                lateralState.mode        = 'SHIFTING';
                lateralState.clearCount  = 0;
                fprintf('  [LATERAL] Obstacle detected at [%.1f,%.1f] → SHIFTING %.1fm left\n',...
                    obstaclePos(1), obstaclePos(2), LATERAL_OFFSET);
            end

        case 'SHIFTING'
            % Gradually increase shift
            lateralState.shiftAmount = lateralState.shiftAmount + SHIFT_SPEED;
            lateralState.shiftAmount = min(lateralState.shiftAmount, lateralState.targetShift);

            if ~inLaneObstacle
                lateralState.clearCount = lateralState.clearCount + 1;
                if lateralState.clearCount > CLEAR_THRESHOLD
                    lateralState.mode = 'RETURNING';
                    fprintf('  [LATERAL] Obstacle cleared → RETURNING to lane\n');
                end
            else
                lateralState.clearCount = 0;
            end

        case 'RETURNING'
            % Gradually reduce shift back to 0
            lateralState.shiftAmount = lateralState.shiftAmount - SHIFT_SPEED;
            if lateralState.shiftAmount <= 0.1
                lateralState.shiftAmount = 0;
                lateralState.mode        = 'CLEAR';
                fprintf('  [LATERAL] Returned to original lane\n');
            end
            % If new obstacle appears, re-shift
            if inLaneObstacle
                lateralState.mode       = 'SHIFTING';
                lateralState.clearCount = 0;
            end
    end

    %% Apply lateral shift to remaining waypoints
    shift = lateralState.shiftAmount;

    if abs(shift) < 0.05
        shiftedWaypoints = wayPoints;
        return;
    end

    % Lateral offset = perpendicular to current heading
    perpX = -sin(egoYaw);  % perpendicular direction x
    perpY =  cos(egoYaw);  % perpendicular direction y

    shiftedWaypoints = wayPoints;
    for i = 1:size(wayPoints,1)
        ahead_dist = (wayPoints(i,1)-egoPos(1))*cos(egoYaw) + ...
                     (wayPoints(i,2)-egoPos(2))*sin(egoYaw);
        if ahead_dist > 0
            % Taper: full shift immediately ahead, fade out at 40m
            taper = min(1.0, max(0, 1 - (ahead_dist-5)/35));
            shiftedWaypoints(i,1) = wayPoints(i,1) + perpX * shift * taper;
            shiftedWaypoints(i,2) = wayPoints(i,2) + perpY * shift * taper;
        end
    end
end
