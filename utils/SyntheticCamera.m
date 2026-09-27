function frame = SyntheticCamera(scenario, egoPos, frameSize)
% SyntheticCamera - Generate a synthetic top-down camera frame from scenario
%
% Renders a simple overhead view of the scenario centred on the ego vehicle.
% Used as proxy "camera frame" input to YOLO, DeepLab, and CNN models.
%
% Inputs:
%   scenario  - drivingScenario object
%   egoPos    - [x y z] ego world position
%   frameSize - [height width] output image size (e.g. [120 160])
%
% Output:
%   frame     - uint8 [H x W x 3] RGB image

    % Get all actor positions
    ap  = actorPoses(scenario);
    n   = numel(ap);

    % Canvas covers ±40m around ego
    RANGE  = 40;
    H      = frameSize(1);
    W      = frameSize(2);

    % Dark road background
    canvas = uint8(repmat(reshape([30 30 30],[1 1 3]), H, W));

    % Draw a simple road strip (horizontal band in middle)
    roadTop    = round(H * 0.35);
    roadBottom = round(H * 0.65);
    canvas(roadTop:roadBottom, :, 1) = 60;
    canvas(roadTop:roadBottom, :, 2) = 60;
    canvas(roadTop:roadBottom, :, 3) = 60;

    % World-to-image coordinate transform
    w2i = @(wx, wy) deal( ...
        round((wx - egoPos(1) + RANGE) / (2*RANGE) * W), ...
        round((wy - egoPos(2) + RANGE) / (2*RANGE) * H) );

    % Draw each actor as a coloured rectangle
    for k = 1:n
        p  = ap(k).Position;
        [px, py] = w2i(p(1), p(2));

        % Skip if out of frame
        if px < 3 || px > W-3 || py < 3 || py > H-3; continue; end

        % Colour by ClassID (approximate from ActorID heuristic)
        col = ActorColor(ap(k));

        % Draw 6x4 box
        r1 = max(1,py-3); r2 = min(H,py+3);
        c1 = max(1,px-2); c2 = min(W,px+2);
        canvas(r1:r2, c1:c2, 1) = col(1);
        canvas(r1:r2, c1:c2, 2) = col(2);
        canvas(r1:r2, c1:c2, 3) = col(3);
    end

    % Mark ego as bright blue box at centre
    ey = round(H/2); ex = round(W/2);
    r1=max(1,ey-4); r2=min(H,ey+4); c1=max(1,ex-3); c2=min(W,ex+3);
    canvas(r1:r2,c1:c2,1)=50;
    canvas(r1:r2,c1:c2,2)=130;
    canvas(r1:r2,c1:c2,3)=255;

    frame = canvas;
end


function col = ActorColor(actorPose)
% Returns RGB color based on actor properties (heuristic)
    % We only have Position and ActorID from actorPoses
    % Use ActorID mod to guess type (rough heuristic)
    id = actorPose.ActorID;
    switch mod(id, 5)
        case 0; col = [255 200  50];  % yellow  = pedestrian
        case 1; col = [200  80  80];  % red     = vehicle
        case 2; col = [100 200 100];  % green   = bicycle
        case 3; col = [150  80 200];  % purple  = auto
        case 4; col = [160 100  60];  % brown   = cattle
        otherwise; col = [200 200 200];
    end
end
