function wp = GetRemainingWaypoints(wayPoints, egoPos)
% GetRemainingWaypoints - Return waypoints strictly ahead of ego
%
% Fixes the smoothTrajectory snap-back bug:
%   Instead of passing a waypoint BEHIND the current ego position
%   as the first point (which causes MATLAB to snap ego backwards),
%   we prepend the CURRENT ego position as the trajectory start.
%
% Inputs:
%   wayPoints - Nx3 original waypoint matrix
%   egoPos    - [x y z] current ego world position
%
% Output:
%   wp - Mx3 remaining waypoints with egoPos prepended
%        Returns [] if ego has passed all waypoints

    % Find waypoints strictly ahead of ego (along x-axis by default)
    % Use the dominant axis of the path for better generality
    pathRange = wayPoints(end,:) - wayPoints(1,:);
    [~, domAxis] = max(abs(pathRange));

    ahead = wayPoints(:, domAxis) > egoPos(domAxis) + 0.5;

    if ~any(ahead)
        wp = [];   % ego has passed all waypoints (goal reached)
        return;
    end

    remaining = wayPoints(ahead, :);

    % Prepend current ego position so trajectory starts exactly here
    % This prevents MATLAB snapping ego back to a past waypoint
    wp = [egoPos(1,1:3); remaining];
end
