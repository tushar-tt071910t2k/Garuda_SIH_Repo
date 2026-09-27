function logger = MetricsLogger(scenarioName)
% MetricsLogger - Initialize a metrics logging struct for a scenario
%
% Usage:
%   logger = MetricsLogger('Scenario1_VillageRoad');
%   logger = LogStep(logger, speed, steerAngle, minDist, status);
%   PrintSummary(logger);

    logger.ScenarioName       = scenarioName;
    logger.Speeds             = [];
    logger.SteerAngles        = [];
    logger.MinDistances       = [];
    logger.StatusHistory      = {};
    logger.ReplanCount        = 0;
    logger.BrakingCount       = 0;
    logger.StopCount          = 0;
    logger.StartTime          = tic;
    logger.GoalReached        = false;
    logger.CollisionOccurred  = false;
    logger.ElapsedTime        = 0;
end
