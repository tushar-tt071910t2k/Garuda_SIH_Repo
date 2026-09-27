function [safeVelocity, avoidanceStatus, minDist] = CollisionAvoidance(egoPose, actorPoses, nominalVelocity)
% CollisionAvoidance - Distance-based threat detection and speed control
%
% Inputs:
%   egoPose        - [x, y, yaw] of ego vehicle
%   actorPoses     - struct array from targetPoses()
%   nominalVelocity- Desired speed from controller (m/s)
%
% Outputs:
%   safeVelocity   - Commanded speed after avoidance logic (m/s)
%   avoidanceStatus- String: 'CRUISE', 'CAUTION', 'BRAKING', 'STOPPED'
%   minDist        - Minimum distance to nearest threat (m)

    EMERGENCY_STOP_DIST  = 5.0;   % m  - Hard stop
    BRAKING_DIST         = 12.0;  % m  - Begin braking
    CAUTION_DIST         = 20.0;  % m  - Reduce speed

    safeVelocity    = nominalVelocity;
    avoidanceStatus = 'CRUISE';
    minDist         = Inf;

    if isempty(actorPoses)
        return;
    end

    egoPos = egoPose(1:2);

    for i = 1:numel(actorPoses)
        actorPos = actorPoses(i).Position(1:2);
        dist     = norm(egoPos - actorPos);

        % Dimensional Consistency: Explicitly slice and force 1x2 row vectors
        actorPos2D = reshape(actorPoses(i).Position(1:2), 1, 2);
        egoPos2D   = reshape(egoPose(1:2), 1, 2);
        
        toActor    = actorPos2D - egoPos2D;
        egoHeading = [cos(egoPose(3)), sin(egoPose(3))]; % 1x2 row vector
        
        % Normalize and calculate dot product safely
        dirToActor = toActor / (norm(toActor) + 1e-6);
        dotProd    = dot(dirToActor, egoHeading);
        
        if dotProd < 0.85  % ~31 degrees (narrowed from 60)
            continue;
        end
        
        % Lateral distance to obstacle
        latDist = abs(toActor(1)*egoHeading(2) - toActor(2)*egoHeading(1));
        if latDist > 1.5 % If obstacle is safely to the side, ignore it!
            continue;
        end

        if dist < minDist
            minDist = dist;
        end

        if dist < EMERGENCY_STOP_DIST
            safeVelocity    = 0;
            avoidanceStatus = 'STOPPED';
            return;

        elseif dist < BRAKING_DIST
            % Linear speed reduction
            brakeFactor  = (dist - EMERGENCY_STOP_DIST) / ...
                           (BRAKING_DIST - EMERGENCY_STOP_DIST);
            proposedSpeed = nominalVelocity * brakeFactor;
            if proposedSpeed < safeVelocity
                safeVelocity    = proposedSpeed;
                avoidanceStatus = 'BRAKING';
            end

        elseif dist < CAUTION_DIST
            proposedSpeed = nominalVelocity * 0.6;
            if proposedSpeed < safeVelocity
                safeVelocity    = proposedSpeed;
                avoidanceStatus = 'CAUTION';
            end
        end
    end

    if strcmp(avoidanceStatus, 'CRUISE') && minDist < Inf
        % No threat but actors present
        avoidanceStatus = 'CRUISE';
    end
end
