function logger = LogStep(logger, speed, steerAngle, minDist, status)
% LogStep - Record one timestep of metrics into a logger struct
%
% Inputs:
%   logger     - struct from MetricsLogger()
%   speed      - current speed (m/s)
%   steerAngle - steering command (omega)
%   minDist    - min clearance distance (m)
%   status     - string status ('CRUISE', 'BRAKING', etc.)

    logger.Speeds(end+1)        = speed;
    logger.SteerAngles(end+1)   = steerAngle;
    logger.MinDistances(end+1)  = minDist;
    logger.StatusHistory{end+1} = status;

    % Count events
    if strcmp(status, 'BRAKING')
        logger.BrakingCount = logger.BrakingCount + 1;
    elseif strcmp(status, 'STOPPED') || strcmp(status, 'WAITING-CATTLE')
        logger.StopCount = logger.StopCount + 1;
    end

    % Collision if min distance < 1 m
    if minDist < 1.0
        logger.CollisionOccurred = true;
    end
end
