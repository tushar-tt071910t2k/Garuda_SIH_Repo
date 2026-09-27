function ReplanTrajectory(ego, wayPoints, egoPos, v_final, prevSpeedRef, stepRef)
% ReplanTrajectory — Safe per-frame trajectory update
%
% FIXES the teleport/snap bug:
%   - Only replans when speed changes significantly (> 0.4 m/s)
%   - Always uses STRICTLY AHEAD waypoints only
%   - Prepends CURRENT ego position so trajectory starts exactly here
%   - Rate-limits replanning to every 5 frames
%
% Usage (in scenario loop):
%   persistent pSpd pStep;
%   if isempty(pSpd); pSpd=nominalSpeed; pStep=0; end
%   ReplanTrajectory(ego, wayPoints, egoPos, v_final, pSpd, pStep);
%   pSpd=v_final; pStep=pStep+1;

    SPEED_THRESHOLD = 0.4;  % m/s change needed to trigger replan
    REPLAN_EVERY    = 5;    % frames between replans (at 0.05s → every 0.25s)

    speedChanged = abs(v_final - prevSpeedRef) > SPEED_THRESHOLD;
    timeToReplan = mod(stepRef, REPLAN_EVERY) == 0;

    if ~(speedChanged || timeToReplan); return; end

    % Only waypoints STRICTLY ahead of ego (prevent backward jump)
    [~, domAxis] = max(abs(wayPoints(end,:) - wayPoints(1,:)));
    ahead = wayPoints(:, domAxis) > egoPos(domAxis) + 0.3;

    if ~any(ahead); return; end  % reached goal

    remaining = [egoPos(1:3); wayPoints(ahead, :)];  % anchor at current pos
    nPts = size(remaining, 1);
    speeds = max(v_final, 0.1) * ones(nPts, 1);

    try; smoothTrajectory(ego, remaining, speeds); catch; end
end
