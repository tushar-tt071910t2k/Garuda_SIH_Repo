function [safeSpeed, maxCurvature, kinFeasible] = VelocityProfiler(wayPoints, egoPos, nominalSpeed, collisionSpeed)
% VelocityProfiler - Curvature-based speed limiter
%
% Ported from Python path_planner_pkg.py output field 'projective_curvature_ref'.
% Adds a kinematic feasibility layer on top of CollisionAvoidance:
%   v_max = sqrt(a_lat_max / curvature)
% The final safe speed is min(collisionSpeed, curvature_speed_limit).
%
% Inputs:
%   wayPoints      - Nx3 waypoint matrix (world frame)
%   egoPos         - [x y z] current ego position (used to find lookahead)
%   nominalSpeed   - desired cruise speed (m/s)
%   collisionSpeed - speed from CollisionAvoidance (m/s)
%
% Outputs:
%   safeSpeed    - final commanded speed (m/s)
%   maxCurvature - maximum curvature found on upcoming path (1/m)
%   kinFeasible  - true if path is within kinematic limits
%
% Vehicle constants (matches SIH ego vehicle specs):
%   wheelbase      = 2.7 m
%   max_steer      = 0.52 rad (~30 deg)
%   -> kappa_max   = tan(max_steer) / wheelbase = 0.202 /m
%   max_lat_accel  = 3.0 m/s^2

    A_LAT_MAX   = 3.0;    % m/s^2 — max lateral acceleration
    KAPPA_MAX   = 0.202;  % 1/m   — max curvature (tan(30deg)/2.7m)
    MIN_SPEED   = 0.5;    % m/s   — never command below this

    safeSpeed    = collisionSpeed;
    maxCurvature = 0;
    kinFeasible  = true;

    %% Guard: need at least 3 waypoints to compute curvature
    nWP = size(wayPoints, 1);
    if nWP < 3
        safeSpeed = max(MIN_SPEED, min(collisionSpeed, nominalSpeed));
        return;
    end

    %% Find waypoints ahead of ego (within planning horizon of 20 m)
    [~, domAxis] = max(abs(wayPoints(end,:) - wayPoints(1,:)));
    ahead = wayPoints(:, domAxis) > egoPos(domAxis);
    wpAhead = wayPoints(ahead, 1:2);   % only X,Y needed for curvature

    if size(wpAhead, 1) < 3
        safeSpeed = max(MIN_SPEED, min(collisionSpeed, nominalSpeed));
        return;
    end

    %% Compute curvature at each waypoint using finite differences
    %  kappa = |x'*y'' - y'*x''| / (x'^2 + y'^2)^1.5
    x = wpAhead(:, 1);
    y = wpAhead(:, 2);

    % First and second derivatives (central differences)
    dx  = gradient(x);
    dy  = gradient(y);
    d2x = gradient(dx);
    d2y = gradient(dy);

    ds2 = dx.^2 + dy.^2 + 1e-8;   % avoid div/0
    kappa = abs(dx .* d2y - dy .* d2x) ./ (ds2 .^ 1.5);

    maxCurvature = max(kappa);

    %% Kinematic feasibility check (mirrors Python's kinematically_verified)
    if maxCurvature > KAPPA_MAX
        kinFeasible = false;
        % Don't hard-reject — just reduce speed so curvature is survivable
    end

    %% Curvature-based speed cap: v <= sqrt(a_lat / kappa)
    if maxCurvature > 1e-4
        curvatureSpeedLimit = sqrt(A_LAT_MAX / maxCurvature);
    else
        curvatureSpeedLimit = nominalSpeed;   % straight road: no limit
    end

    %% Final speed: minimum of all constraints
    safeSpeed = min([collisionSpeed, curvatureSpeedLimit, nominalSpeed]);
    safeSpeed = max(safeSpeed, MIN_SPEED);
end
