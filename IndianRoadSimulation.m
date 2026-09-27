% Scenario 1: Unmarked Village Road with Collision Braking, Telemetry & Chase Plot
% Designed for Smart India Hackathon (SIH)

% 1. Initialize Scenario and Narrow Unmarked Road
scenario = drivingScenario('SampleTime', 0.1);

% Define a winding village road center with single lane spec
roadCenters = [0 0 0; 40 0 0; 80 5 0; 120 10 0; 160 10 0];
road(scenario, roadCenters, 'Lanes', lanespec(1, 'Width', 3.5));

% 2. Add Ego Vehicle
egoVehicle = vehicle(scenario, ...
    'ClassID', 1, ...
    'Length', 4.0, ...
    'Width', 1.7, ...
    'Height', 1.5);

% Define waypoints for the Pure Pursuit controller to follow
wayPoints = [0 0 0; 40 0 0; 80 5 0; 120 10 0; 150 10 0];

% 3. Add Dynamic Pedestrian Crossing Unexpectedly
pedestrian = actor(scenario, ...
    'ClassID', 4, ...
    'Length', 0.5, ...
    'Width', 0.5, ...
    'Height', 1.7);

% Program the pedestrian to cross the road path at x = 70 meters
trajectory(pedestrian, [70 -4 0; 70 4 0], 1.5); 

% 4. Setup Path Controller
controller = controllerPurePursuit('Waypoints', wayPoints(:,1:2));
controller.DesiredLinearVelocity = 6;
controller.MaxCurvature = 1.2;
controller.LookaheadDistance = 2.5;

% 5. Open Built-in 3D Chase Camera View (Using valid properties only)
chasePlot(egoVehicle, 'Meshes', 'on', 'RoadCenters', 'on', 'ViewHeight', 3);

% 6. Continuous Running Mode Loop (Repeats endlessly until you press Ctrl+C or close)
while true
    restart(scenario); % Resets vehicle and pedestrian back to start position
    
    running = true;
    while running && advance(scenario)
        % Get current ego vehicle position and orientation
        pose = targetPoses(egoVehicle);
        if isempty(pose)
            break;
        end
        currentPose = [pose.Position(1) pose.Position(2) pose.Yaw];
        
        % Compute steering and speed commands from controller
        [v, omega] = controller(currentPose);
        
        % --- COLLISION AVOIDANCE & TELEMETRY LOGIC ---
        pedPose = targetPoses(pedestrian);
        statusText = 'CRUISE MODE';
        textColor = [0 0.5 0]; % Dark Green
        
        if ~isempty(pedPose)
            % Calculate distance between ego vehicle and pedestrian
            dist = norm(currentPose(1:2) - pedPose.Position(1:2));
            
            % If pedestrian is within 12 meters ahead, apply emergency brake
            if dist < 12 && pedPose.Position(1) >= currentPose(1)
                v = 0; % Force vehicle speed to zero
                statusText = 'OBSTACLE DETECTED - BRAKING!';
                textColor = [0.8 0 0]; % Red
            end
        end
        % ---------------------------------------------
        
        % Move ego vehicle dynamically based on controller and braking output
        move(egoVehicle, [v*0.1, 0, 0], [0, 0, omega*0.1]);
        
        % Update live telemetry status header directly on the active chase plot window
        title(sprintf('Speed: %.1f m/s | Status: %s', v, statusText), ...
              'Color', textColor, 'FontSize', 11, 'FontWeight', 'bold');
              
        drawnow limitrate;
        
        % Stop condition when simulation finishes its route
        running = ~isDone(scenario);
    end
end