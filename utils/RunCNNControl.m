function [steer, speed, confidence] = RunCNNControl(frame, net, frameSize, nominalSpeed)
% RunCNNControl - CNN behavioral clone inference
% Returns steering angle + speed command from camera frame
% Handles: trained cnnNet, or rule-based fallback

    if nargin < 4; nominalSpeed = 6; end

    steer = 0; speed = nominalSpeed; confidence = 0;

    if isempty(net)
        %% Rule-based fallback: keep straight, nominal speed
        confidence = 0;
        return;
    end

    try
        img = imresize(frame, frameSize(1:2));

        % Predict [steer, speed]
        pred = predict(net, img);
        if numel(pred) >= 2
            steer      = double(pred(1));
            speed      = double(pred(2));
            speed      = max(0.5, min(nominalSpeed * 1.1, speed));
            steer      = max(-0.5, min(0.5, steer));
            confidence = 0.75;   % synthetic-trained → moderate confidence
        end
    catch
        confidence = 0;
    end
end
