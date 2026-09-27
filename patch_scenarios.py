import os
import glob

OLD_BLOCK = """    % --- BEGIN ADVANCED COLLISION & HALT LOGIC ---
    if strcmp(status, 'STOPPED') || (strcmp(status, 'BRAKING') && minDist < 2.0)
        fprintf('[EMERGENCY] Perception triggered hard stop at T=%.2fs! Halting scenario safely.\\n', scenario.SimulationTime);
        break; 
    end
    
    dt = scenario.SampleTime;
    egoRadius = max(ego.Length, ego.Width) / 2.0;
    
    for actIdx = 1:numel(scenario.Actors)
        act = scenario.Actors(actIdx);
        if act.ActorID == ego.ActorID; continue; end
        
        centerDist = norm(egoPos(1:2) - act.Position(1:2));
        actRadius = max(act.Length, act.Width) / 2.0;
        physClearance = centerDist - (egoRadius + actRadius);
        
        relSpeed = norm(egoVel(1:2) - act.Velocity(1:2));
        midFrameTravel = relSpeed * dt;
        
        if physClearance <= midFrameTravel
            metrics.CollisionOccurred = true;
            fprintf('💥 [CRASH] Physical collision detected with Actor %d at T=%.2fs!\\n', act.ActorID, scenario.SimulationTime);
            break;
        end
    end
    % --- END ADVANCED COLLISION & HALT LOGIC ---"""

NEW_BLOCK = """    % --- BEGIN ADVANCED COLLISION & HALT LOGIC ---
    persistent isHalted;
    if isempty(isHalted); isHalted = false; end
    
    if (strcmp(status, 'STOPPED') || (strcmp(status, 'BRAKING') && minDist < 2.0)) && ~isHalted
        fprintf('[EMERGENCY] Perception triggered hard stop at T=%.2fs!\\n', scenario.SimulationTime);
        trajectory(ego, [egoPos(1:2) 0; egoPos(1:2) + [0.01*cos(egoYaw), 0.01*sin(egoYaw), 0]], [0; 0]);
        isHalted = true;
    elseif strcmp(status, 'CRUISE') && isHalted
        fprintf('[RESUME] Path clear at T=%.2fs! Resuming...\\n', scenario.SimulationTime);
        if exist('wayPoints', 'var'); wps = wayPoints; elseif exist('wpNormal', 'var'); wps = wpNormal; else; wps = []; end
        if ~isempty(wps)
            % Find waypoints ahead
            dists = vecnorm(wps(:,1:2)' - egoPos(1:2)');
            [~, closestIdx] = min(dists);
            remWps = wps(closestIdx:end, :);
            if size(remWps, 1) < 2; remWps = [egoPos(1:2) 0; egoPos(1:2) + [10*cos(egoYaw), 10*sin(egoYaw), 0]]; end
            trajectory(ego, remWps, nominalSpeed * ones(size(remWps,1),1));
        end
        isHalted = false;
    end
    
    dt = scenario.SampleTime;
    crashDetected = false;
    for actIdx = 1:numel(scenario.Actors)
        act = scenario.Actors(actIdx);
        if act.ActorID == ego.ActorID; continue; end
        
        % OBB / AABB Approximation in Ego Frame
        dx = act.Position(1) - egoPos(1);
        dy = act.Position(2) - egoPos(2);
        
        cosY = cos(egoYaw); sinY = sin(egoYaw);
        localX = dx * cosY + dy * sinY;
        localY = -dx * sinY + dy * cosY;
        
        clearanceX = abs(localX) - (ego.Length/2 + act.Length/2);
        clearanceY = abs(localY) - (ego.Width/2 + act.Width/2);
        
        relSpeed = norm(egoVel(1:2) - act.Velocity(1:2));
        midFrameTravel = max(relSpeed * dt, 0.1); % ensure minimum epsilon
        
        if clearanceX <= midFrameTravel && clearanceY <= midFrameTravel
            metrics.CollisionOccurred = true;
            fprintf('💥 [CRASH] Physical collision with Actor %d at T=%.2fs!\\n', act.ActorID, scenario.SimulationTime);
            crashDetected = true;
            break;
        end
    end
    
    if crashDetected
        break; % Break the while loop
    end
    % --- END ADVANCED COLLISION & HALT LOGIC ---"""

scenario_files = glob.glob('e:/sih_matlab/scenarios/*.m')
for filepath in scenario_files:
    with open(filepath, 'r', encoding='utf-8') as f:
        content = f.read()
    
    if OLD_BLOCK in content:
        new_content = content.replace(OLD_BLOCK, NEW_BLOCK)
        with open(filepath, 'w', encoding='utf-8') as f:
            f.write(new_content)
        print(f"Patched {os.path.basename(filepath)}")
    else:
        print(f"Block not found in {os.path.basename(filepath)}")
