classdef SafePathPlanner < handle
    % SafePathPlanner MATLAB Port of the C-space & Graph/Beam-Search planner.
    
    properties
        resolution
        x_min; x_max; z_min; z_max
        grid_w; grid_h
        
        ego_width; ego_length
        
        dynamic_uncertainty; static_uncertainty; additional_buffer
        min_confidence
        
        required_dynamic_clearance
        required_static_clearance
        required_road_clearance
        
        planning_horizon
        graph_goal
        connector_candidates
        beam_width
        max_allowed_deviation
    end
    
    methods
        function obj = SafePathPlanner(config)
            if nargin < 1 || isempty(config)
                config = struct();
            end
            
            % Default configuration values
            obj.resolution = get_field(config, 'resolution', 0.01);
            obj.x_min = get_field(config, 'x_min', -0.5);
            obj.x_max = get_field(config, 'x_max', 1.5);
            obj.z_min = get_field(config, 'z_min', 0.8);
            obj.z_max = get_field(config, 'z_max', 4.0);
            
            obj.grid_w = round((obj.x_max - obj.x_min) / obj.resolution);
            obj.grid_h = round((obj.z_max - obj.z_min) / obj.resolution);
            
            obj.ego_width = get_field(config, 'ego_width', 0.15);
            obj.ego_length = get_field(config, 'ego_length', 0.30);
            
            obj.dynamic_uncertainty = get_field(config, 'dynamic_uncertainty', 0.100);
            obj.static_uncertainty = get_field(config, 'static_uncertainty', 0.050);
            obj.additional_buffer = get_field(config, 'additional_buffer', 0.025);
            obj.min_confidence = get_field(config, 'min_confidence', 0.50);
            
            obj.required_dynamic_clearance = get_field(config, 'required_dynamic_clearance', 0.200);
            obj.required_static_clearance = get_field(config, 'required_static_clearance', 0.125);
            obj.required_road_clearance = get_field(config, 'required_road_clearance', 0.075);
            
            obj.planning_horizon = get_field(config, 'planning_horizon', 3.4);
            obj.graph_goal = get_field(config, 'graph_goal', 3.8);
            obj.connector_candidates = get_field(config, 'connector_candidates', 31);
            obj.beam_width = get_field(config, 'beam_width', 20);
            obj.max_allowed_deviation = get_field(config, 'max_allowed_deviation', 0.050);
        end
        
        function result = planFromBEV(obj, road_grid, static_footprints, dynamic_footprints, ego_state)
            if nargin < 5 || isempty(ego_state)
                ego_state = struct('x', 0.5 * (obj.x_min + obj.x_max), 'z', obj.z_min);
            end
            
            diagnostics = struct();
            
            [safe_free, cspace_obs, dist_dyn, dist_stat, dist_road] = obj.buildCSpace(road_grid, static_footprints, dynamic_footprints);
            
            if isempty(safe_free)
                result = obj.buildFail('NO_SAFE_ROUTE', false, false, diagnostics, 0, 0, 0);
                return;
            end
            
            [nodes, graph_exists] = obj.runBeamSearch(safe_free, dist_dyn, dist_stat, dist_road, ego_state);
            
            if ~graph_exists
                result = obj.buildFail('NO_SAFE_ROUTE', false, false, diagnostics, 0, 0, 0);
                return;
            end
            
            Z_g = [nodes.z];
            X_g = [nodes.chosen_x];
            
            Z_dense = linspace(Z_g(1), Z_g(end), max(500, length(Z_g)*10));
            
            try
                X_dense = pchip(Z_g, X_g, Z_dense);
                X_lin = interp1(Z_g, X_g, Z_dense, 'linear');
                if max(abs(X_dense - X_lin)) > obj.max_allowed_deviation
                    X_dense = X_lin;
                end
            catch
                X_dense = interp1(Z_g, X_g, Z_dense, 'linear');
            end
            
            idx_z = round((Z_dense - obj.z_min) / obj.resolution) + 1;
            idx_x = round((X_dense - obj.x_min) / obj.resolution) + 1;
            
            idx_z = max(1, min(obj.grid_h, idx_z));
            idx_x = max(1, min(obj.grid_w, idx_x));
            
            coords = sub2ind([obj.grid_h, obj.grid_w], idx_z, idx_x);
            
            d_dyn = dist_dyn(coords);
            d_stat = dist_stat(coords);
            d_road = dist_road(coords);
            
            min_d = min(d_dyn);
            min_s = min(d_stat);
            min_r = min(d_road);
            
            cspace_valid = true;
            for i = 1:length(idx_x)
                if cspace_obs(idx_z(i), idx_x(i)) || road_grid(idx_z(i), idx_x(i)) == 0
                    cspace_valid = false;
                    break;
                end
            end
            
            verified_safe = cspace_valid && ...
                min_d >= obj.required_dynamic_clearance - 1e-4 && ...
                min_s >= obj.required_static_clearance - 1e-4 && ...
                min_r >= obj.required_road_clearance - 1e-4;
                
            diagnostics.graph_exists = graph_exists;
            diagnostics.verified_safe = verified_safe;
            
            if ~verified_safe
                result = obj.buildFail('NO_SAFE_ROUTE', graph_exists, false, diagnostics, min_d, min_s, min_r);
                return;
            end
            
            dx = gradient(X_dense);
            dz = gradient(Z_dense);
            ds = hypot(dx, dz) + 1e-6;
            heading_ref = atan2(dx, dz);
            
            dx2 = gradient(dx);
            dz2 = gradient(dz);
            proj_curvature = abs(dx .* dz2 ./ ds - dz .* dx2 ./ ds) ./ (ds.^3);
            
            result = struct(...
                'status', 'PLANNER_SAFE', ...
                'route_exists', true, ...
                'verified_safe', true, ...
                'kinematically_verified', false, ...
                'X_ref', X_dense, ...
                'Z_ref', Z_dense, ...
                'heading_ref', heading_ref, ...
                'projective_curvature_ref', proj_curvature, ...
                'requiredDynamicClearance', obj.required_dynamic_clearance, ...
                'requiredStaticClearance', obj.required_static_clearance, ...
                'requiredRoadClearance', obj.required_road_clearance, ...
                'minDynamicClearance', min_d, ...
                'minStaticClearance', min_s, ...
                'minRoadClearance', min_r, ...
                'planningHorizon', obj.planning_horizon, ...
                'graphGoal', obj.graph_goal, ...
                'diagnostics', diagnostics ...
            );
        end
        
        function [safe_free, cspace_obstacle, dist_dyn, dist_stat, dist_road] = buildCSpace(obj, road_grid, static_footprints, dynamic_footprints)
            if isempty(road_grid) || size(road_grid, 1) ~= obj.grid_h || size(road_grid, 2) ~= obj.grid_w
                safe_free = []; cspace_obstacle = []; dist_dyn = []; dist_stat = []; dist_road = [];
                return;
            end
            
            raw_dyn = zeros(obj.grid_h, obj.grid_w, 'uint8');
            raw_stat = zeros(obj.grid_h, obj.grid_w, 'uint8');
            
            raw_dyn = rasterizePolys(obj, raw_dyn, dynamic_footprints);
            raw_stat = rasterizePolys(obj, raw_stat, static_footprints);
            
            ego_half = obj.ego_width / 2.0;
            inf_dyn_m = ego_half + obj.dynamic_uncertainty + obj.additional_buffer;
            inf_stat_m = ego_half + obj.static_uncertainty + obj.additional_buffer;
            
            k_dyn = max(3, round(inf_dyn_m / obj.resolution) * 2 + 1);
            k_stat = max(3, round(inf_stat_m / obj.resolution) * 2 + 1);
            
            se_dyn = strel('square', k_dyn);
            se_stat = strel('square', k_stat);
            
            cspace_dyn = imdilate(raw_dyn > 0, se_dyn);
            cspace_stat = imdilate(raw_stat > 0, se_stat);
            cspace_obstacle = cspace_dyn | cspace_stat;
            
            safe_free = (road_grid > 0) & ~cspace_obstacle;
            
            if any(raw_dyn(:) > 0)
                dist_dyn = bwdist(raw_dyn > 0) * obj.resolution;
            else
                dist_dyn = ones(obj.grid_h, obj.grid_w) * 10.0;
            end
            
            if any(raw_stat(:) > 0)
                dist_stat = bwdist(raw_stat > 0) * obj.resolution;
            else
                dist_stat = ones(obj.grid_h, obj.grid_w) * 10.0;
            end
            
            padded_road = padarray(road_grid, [0 1], 0, 'both');
            dist_road_padded = bwdist(~padded_road) * obj.resolution;
            dist_road = dist_road_padded(:, 2:end-1);
        end
        
        function [best_nodes, route_exists] = runBeamSearch(obj, safe_free, dist_dyn, dist_stat, dist_road, ego_state)
            x_arr = obj.x_min + (0:obj.grid_w-1) * obj.resolution;
            z_arr = obj.z_min + (0:obj.grid_h-1) * obj.resolution;
            
            layers_nodes = cell(obj.grid_h, 1);
            for layer_idx = 1:obj.grid_h
                row_cells = safe_free(layer_idx, :);
                free_indices = find(row_cells);
                row_nodes = {};
                if ~isempty(free_indices)
                    splits = find(diff(free_indices) > 1) + 1;
                    intervals = mat2cell(free_indices, 1, diff([0, splits, length(free_indices)]));
                    for j = 1:length(intervals)
                        interval = intervals{j};
                        if length(interval) > 2
                            l = x_arr(interval(1));
                            r = x_arr(interval(end));
                            row_nodes{end+1} = struct('z', z_arr(layer_idx), 'left', l, 'right', r, 'width', r - l); %#ok<AGROW>
                        end
                    end
                end
                layers_nodes{layer_idx} = row_nodes;
            end
            
            min_corridor_width = obj.ego_width + (2.0 * obj.required_road_clearance);
            
            ego_x = ego_state.x;
            ego_z = ego_state.z;
            
            [~, start_idx] = min(abs(z_arr - ego_z));
            
            active_paths = {};
            for j = 1:length(layers_nodes{start_idx})
                node = layers_nodes{start_idx}{j};
                if node.left <= ego_x && ego_x <= node.right && node.width >= min_corridor_width
                    n_copy = node;
                    n_copy.chosen_x = ego_x;
                    active_paths{end+1} = struct('nodes', {n_copy}, 'cost', 0.0, 'prev_dx', 0.0); %#ok<AGROW>
                end
            end
            
            if isempty(active_paths)
                best_nodes = [];
                route_exists = false;
                return;
            end
            
            complete_routes = {};
            for layer_idx = (start_idx + 1):length(layers_nodes)
                row_nodes = layers_nodes{layer_idx};
                next_active = {};
                for p_idx = 1:length(active_paths)
                    path = active_paths{p_idx};
                    last_node = path.nodes(end);
                    
                    for n_idx = 1:length(row_nodes)
                        node = row_nodes{n_idx};
                        if node.width < min_corridor_width
                            continue;
                        end
                        
                        xa = last_node.chosen_x;
                        za = last_node.z;
                        zb = node.z;
                        xb_cands = linspace(node.left, node.right, obj.connector_candidates);
                        
                        best_xb = [];
                        best_cost = inf;
                        
                        for c_idx = 1:length(xb_cands)
                            xb = xb_cands(c_idx);
                            xs = linspace(xa, xb, 10);
                            zs = linspace(za, zb, 10);
                            
                            idx_x = round((xs - obj.x_min) / obj.resolution) + 1;
                            idx_z = round((zs - obj.z_min) / obj.resolution) + 1;
                            
                            idx_x = max(1, min(obj.grid_w, idx_x));
                            idx_z = max(1, min(obj.grid_h, idx_z));
                            
                            feasible = true;
                            min_d_val = inf;
                            min_s_val = inf;
                            min_r_val = inf;
                            
                            for i = 1:10
                                if ~safe_free(idx_z(i), idx_x(i))
                                    feasible = false;
                                    break;
                                end
                                min_d_val = min(min_d_val, dist_dyn(idx_z(i), idx_x(i)));
                                min_s_val = min(min_s_val, dist_stat(idx_z(i), idx_x(i)));
                                min_r_val = min(min_r_val, dist_road(idx_z(i), idx_x(i)));
                            end
                            
                            if feasible && min_d_val >= obj.required_dynamic_clearance && ...
                                    min_s_val >= obj.required_static_clearance && ...
                                    min_r_val >= obj.required_road_clearance
                                
                                lat_trans = abs(xb - xa);
                                smoothness = abs((xb - xa) - path.prev_dx);
                                cost = lat_trans + 2.0 * smoothness - 5.0 * min(min_d_val, 0.3);
                                
                                if isfinite(cost) && cost < best_cost
                                    best_cost = cost;
                                    best_xb = xb;
                                end
                            end
                        end
                        
                        if ~isempty(best_xb)
                            n_copy = node;
                            n_copy.chosen_x = best_xb;
                            new_p = struct('nodes', [path.nodes, n_copy], ...
                                'cost', path.cost + best_cost, ...
                                'prev_dx', best_xb - xa);
                                
                            if node.z >= obj.graph_goal
                                complete_routes{end+1} = new_p; %#ok<AGROW>
                            else
                                next_active{end+1} = new_p; %#ok<AGROW>
                            end
                        end
                    end
                end
                
                if length(next_active) > obj.beam_width
                    costs = cellfun(@(x) x.cost, next_active);
                    [~, sort_idx] = sort(costs);
                    next_active = next_active(sort_idx(1:obj.beam_width));
                end
                active_paths = next_active;
            end
            
            if isempty(complete_routes)
                best_nodes = [];
                route_exists = false;
                return;
            end
            
            costs = cellfun(@(x) x.cost, complete_routes);
            [~, sort_idx] = sort(costs);
            best_path = complete_routes{sort_idx(1)};
            
            best_nodes = best_path.nodes;
            
            zs = [best_nodes.z];
            [~, idx_tgt] = min(abs(zs - obj.planning_horizon));
            best_nodes = best_nodes(1:idx_tgt);
            route_exists = true;
        end
        
        function grid = rasterizePolys(obj, grid, polys)
            if isempty(polys)
                return;
            end
            
            for i = 1:length(polys)
                poly = polys{i};
                if length(poly) < 3
                    continue;
                end
                
                if iscell(poly)
                    x = cellfun(@(p) p(1), poly);
                    z = cellfun(@(p) p(2), poly);
                else
                    x = poly(:, 1);
                    z = poly(:, 2);
                end
                
                gx = round((x - obj.x_min) / obj.resolution) + 1;
                gz = round((z - obj.z_min) / obj.resolution) + 1;
                
                mask = poly2mask(gx, gz, obj.grid_h, obj.grid_w);
                grid(mask) = 1;
            end
        end
        
        function result = buildFail(obj, st, g_e, v_s, diag, md, ms, mr)
            result = struct(...
                'status', st, ...
                'route_exists', g_e, ...
                'verified_safe', v_s, ...
                'kinematically_verified', false, ...
                'X_ref', [], ...
                'Z_ref', [], ...
                'heading_ref', [], ...
                'projective_curvature_ref', [], ...
                'requiredDynamicClearance', obj.required_dynamic_clearance, ...
                'requiredStaticClearance', obj.required_static_clearance, ...
                'requiredRoadClearance', obj.required_road_clearance, ...
                'minDynamicClearance', md, ...
                'minStaticClearance', ms, ...
                'minRoadClearance', mr, ...
                'planningHorizon', obj.planning_horizon, ...
                'graphGoal', obj.graph_goal, ...
                'diagnostics', diag ...
            );
        end
    end
end

function val = get_field(s, field, default)
    if isfield(s, field)
        val = s.(field);
    else
        val = default;
    end
end
