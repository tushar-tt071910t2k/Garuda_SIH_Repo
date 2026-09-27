function poses = GetActorPosesByID(scenario, actorIDs)
% GetActorPosesByID - Get world-frame poses of specific actors by ActorID
%
% Works for BOTH smoothTrajectory and trajectory() actors in R2026a.
% Uses actorPoses(scenario) which works for all actor types.
%
% Inputs:
%   scenario  - drivingScenario object
%   actorIDs  - vector of ActorID values to retrieve
%
% Output:
%   poses - struct array with .Position [1x3] for each requested actor
%           (same order as actorIDs). Empty struct if actor not found.

    allPoses = actorPoses(scenario);   % all actors, world frame
    poses    = struct('Position', {});

    for k = 1:numel(actorIDs)
        found = false;
        for j = 1:numel(allPoses)
            if allPoses(j).ActorID == actorIDs(k)
                poses(k).Position = allPoses(j).Position;
                found = true;
                break;
            end
        end
        if ~found
            poses(k).Position = [Inf Inf Inf];
        end
    end
end
