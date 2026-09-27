function [lateralShift, bestSide, safeVerified, planDiag] = CSpacePlanner(egoPos, egoYaw, actorPoses, cfg, currentShift)
% CSpacePlanner - MATLAB port of Python path_planner_pkg.py
%   CSpaceEngine + GraphBeamPlanner combined.
%
% Operates in a LOCAL ego-centered BEV grid:
%   Z-axis = forward along ego heading
%   X-axis = lateral (positive = LEFT of ego)
%
% Inputs:
%   egoPos     - [x y z] ego position in world frame (m)
%   egoYaw     - ego heading (radians, world frame)
%   actorPoses - struct array with .Position field for each actor
%   cfg        - config struct from cspace_default_config() (optional)
%   currentShift - current lateral shift of the ego from the center line (m)

    if nargin < 5
        currentShift = 0;
    end
    %% Default config
    if nargin < 4 || isempty(cfg)
        cfg = cspace_default_config();
    end

    planDiag = struct('nObstacles', 0, 'minClearance', Inf, ...
                      'beamPaths', 0, 'gridShape', [0 0]);

    %% 1. Build local BEV grid dimensions
    res = cfg.resolution;                            % metres per cell
    nZ  = round(cfg.zMax  / res);                   % forward cells
    nX  = round(2 * cfg.xHalf / res);               % lateral cells
    planDiag.gridShape = [nZ, nX];

    %% 2. Rasterize obstacle footprints onto local grid
    obs_grid = zeros(nZ, nX, 'uint8');
    cosY = cos(egoYaw);
    sinY = sin(egoYaw);
    nObs = 0;

    for k = 1:numel(actorPoses)
        ap = actorPoses(k).Position;

        % Extract absolute velocity if available
        if isfield(actorPoses(k), 'Velocity')
            av = actorPoses(k).Velocity;
        else
            av = [0 0 0];
        end

        % Transform world to ego-local frame
        dx    =  ap(1) - egoPos(1);
        dy    =  ap(2) - egoPos(2);
        Z_loc =  dx * cosY + dy * sinY;   % forward distance
        X_loc = -dx * sinY + dy * cosY;   % lateral offset

        % Transform absolute velocity to ego-local frame
        vx_loc = -av(1) * sinY + av(2) * cosY;
        vz_loc =  av(1) * cosY + av(2) * sinY;

        % Reject actors behind or too far originally
        if Z_loc < 0.3 || Z_loc > cfg.zMax + 2.0; continue; end
        
        nObs = nObs + 1;

        if isfield(actorPoses, 'Width') && ~isempty(actorPoses(k).Width)
            actHalfW = actorPoses(k).Width / 2;
            actHalfL = actorPoses(k).Length / 2;
        else
            actHalfW = cfg.actorHalfWidth;
            actHalfL = cfg.actorHalfLength;
        end

        % Project swept path forward in time (0 to 2 seconds)
        % Ensure step size is small enough to prevent gaps
        % Maximum speed is maybe 20 m/s. 20 * dt < 1.0m => dt < 0.05s
        t_steps = 0 : 0.1 : 3.0; % 30 steps for 3 seconds
        for t = t_steps
            Z_t = Z_loc + vz_loc * t;
            X_t = X_loc + vx_loc * t;

            % Skip out-of-bounds future positions
            if Z_t < 0.3 || Z_t > cfg.zMax + 2.0; continue; end
            if abs(X_t) > cfg.xHalf + 2.0;        continue; end

            % Footprint rectangle in grid indices for this time step
            gz1 = max(1,  round((Z_t - actHalfL) / res));
            gz2 = min(nZ, round((Z_t + actHalfL) / res));
            gx1 = max(1,  round((X_t - actHalfW + cfg.xHalf) / res));
            gx2 = min(nX, round((X_t + actHalfW + cfg.xHalf) / res));

            if gz2 >= gz1 && gx2 >= gx1
                obs_grid(gz1:gz2, gx1:gx2) = 1;
            end
        end
    end

    planDiag.nObstacles = nObs;

    %% 3. Early exit: no obstacles in range -> stay centered
    if nObs == 0
        lateralShift = 0;
        bestSide     = 'CENTER';
        safeVerified = true;
        return;
    end

    %% 4. C-Space Inflation  (mirrors Python ndimage.binary_dilation)
    %    Inflate by: ego_half_width + uncertainty_buffer + additional_buffer
    inf_radius_m     = cfg.egoHalfWidth + cfg.uncertaintyBuffer + cfg.additionalBuffer;
    inf_radius_cells = max(2, ceil(inf_radius_m / res));
    se               = strel('disk', inf_radius_cells);
    cspace           = imdilate(obs_grid, se) > 0;   % true = unsafe

    %% 5. Distance Transform  (mirrors Python ndimage.distance_transform_edt)
    if any(obs_grid(:))
        dist_obs = bwdist(obs_grid) * res;   % metres to nearest obstacle cell
    else
        dist_obs = inf(nZ, nX);
    end

    %% 6. Beam Search: evaluate N_CAND lateral candidates
    % x_cands are now ABSOLUTE shifts relative to the road center
    N_CAND   = cfg.nCandidates;
    x_cands  = linspace(-cfg.xHalf * 0.85, cfg.xHalf * 0.85, N_CAND);
    z_sample = linspace(0.5, cfg.planningHorizon, 20);

    bestCost  = Inf;
    bestX     = 0;
    beamCount = 0;
    bestClear = 0;

    for ci = 1:N_CAND
        xb = x_cands(ci); % Absolute target shift from road center

        % Relative shift needed from ego's current position
        rel_shift = xb - currentShift;
        
        % Ego swerves from its current offset over approx 15m
        % Since X_loc in the grid is relative to Ego, the candidate path 
        % in the grid must also be relative to Ego!
        xs_grid = min(z_sample / 15.0, 1.0) * rel_shift;

        % Convert to grid indices (clamp to valid range)
        iz = min(nZ, max(1, round(z_sample / res)));
        ix = min(nX, max(1, round((xs_grid + cfg.xHalf) / res)));

        % Flat linear index
        linIdx = sub2ind([nZ, nX], iz, ix);

        % (a) Reject if any sample hits inflated C-Space
        if any(cspace(linIdx))
            continue;
        end

        % (b) Minimum clearance along path
        minClear = min(dist_obs(linIdx));

        if minClear < cfg.requiredClearance
            continue;
        end

        % Cost: penalise absolute lateral deviation, penalise changing current shift, reward clearance
        latDev             = abs(xb);
        shiftChangePenalty = 2.5 * abs(rel_shift); % Very strong penalty to prevent lane-flipping and tail swings
        clearBonus         = 3.0 * min(minClear, cfg.requiredClearance * 2.0);
        
        cost = latDev + shiftChangePenalty - clearBonus;

        if cost < bestCost
            bestCost  = cost;
            bestX     = xb;
            bestClear = minClear;
            beamCount = beamCount + 1;
        end
    end

    planDiag.beamPaths    = beamCount;
    planDiag.minClearance = bestClear;

    %% 7. Build output
    if beamCount == 0
        lateralShift = 0;
        bestSide     = 'BLOCKED';
        safeVerified = false;
        return;
    end

    lateralShift = bestX;
    safeVerified = true;

    if     bestX >  cfg.sideThreshold;  bestSide = 'LEFT';
    elseif bestX < -cfg.sideThreshold;  bestSide = 'RIGHT';
    else;                                bestSide = 'CENTER';
    end
end


%% =========================================================================
function cfg = cspace_default_config()
% cspace_default_config  Returns default planner configuration.
% 25 cm resolution: grid = 100x48 = 4800 cells (was 500x240 = 120k at 5cm)
% 25x smaller grid = 25x faster imdilate + bwdist per frame.
    cfg.resolution        = 0.25;   % 25 cm cells
    cfg.zMax              = 55.0;   % look 55 m ahead (m)
    cfg.xHalf             = 6.0;    % +/-6 m lateral range (m)
    cfg.egoHalfWidth      = 0.85;   % half of 1.7 m vehicle (m)
    cfg.actorHalfWidth    = 0.60;   % conservative actor half-width (m)
    cfg.actorHalfLength   = 1.50;   % conservative actor half-length (m)
    cfg.uncertaintyBuffer = 0.30;   % positional uncertainty (m)
    cfg.additionalBuffer  = 0.20;   % extra safety margin (m)
    cfg.requiredClearance = 0.40;   % minimum path clearance (m)
    cfg.nCandidates       = 11;     % lateral beam candidates
    cfg.planningHorizon   = 45.0;   % forward planning distance (m)
    cfg.sideThreshold     = 0.10;   % min shift to call LEFT/RIGHT vs CENTER
end
