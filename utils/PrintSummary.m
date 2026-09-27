function PrintSummary(logger)
% PrintSummary - Print scenario metrics to command window
%
% Input:
%   logger - struct from MetricsLogger(), populated via LogStep()

    elapsedTime = toc(logger.StartTime);

    % Safe RMS computation
    if numel(logger.SteerAngles) > 1
        smoothness = rms(diff(logger.SteerAngles));
    else
        smoothness = 0;
    end

    fprintf('\n========================================\n');
    fprintf(' METRICS: %s\n', logger.ScenarioName);
    fprintf('========================================\n');
    fprintf(' Sim Wall Time     : %.2f s\n',  elapsedTime);
    fprintf(' Avg Speed         : %.2f m/s\n', mean(logger.Speeds));
    fprintf(' Max Speed         : %.2f m/s\n', max(logger.Speeds));
    fprintf(' Path Smoothness   : %.4f rad (RMS steer change)\n', smoothness);

    if ~isempty(logger.MinDistances) && any(isfinite(logger.MinDistances))
        finiteVals = logger.MinDistances(isfinite(logger.MinDistances));
        fprintf(' Min Clearance     : %.2f m\n', min(finiteVals));
    else
        fprintf(' Min Clearance     : N/A (no obstacles detected)\n');
    end

    fprintf(' Braking Events    : %d\n', logger.BrakingCount);
    fprintf(' Full Stop Events  : %d\n', logger.StopCount);
    fprintf(' Collision         : %s\n', boolStr(logger.CollisionOccurred, 'YES ❌', 'NO ✅'));
    fprintf(' Goal Reached      : %s\n', boolStr(logger.GoalReached,      'YES ✅', 'NO ❌'));
    fprintf('========================================\n\n');
end


function str = boolStr(cond, trueStr, falseStr)
    if cond; str = trueStr; else; str = falseStr; end
end
