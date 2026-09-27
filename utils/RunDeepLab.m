function roadFraction = RunDeepLab(frame, net, inputSize)
% RunDeepLab - Estimate drivable road fraction from camera frame
% Handles: trained segmentation CNN, or geometric fallback

    if nargin < 3; inputSize = [120 160 3]; end

    if isempty(net)
        %% Geometric fallback: bottom 45% of frame = road heuristic
        H = size(frame,1);
        roadFraction = 0.45 + 0.15 * (mean(mean(double(frame(round(H*0.6):end,:,:)))) / 255 - 0.5);
        roadFraction = max(0.2, min(0.9, roadFraction));
        return;
    end

    try
        img = imresize(frame, inputSize(1:2));

        % Check if it's the fast segNet (regression, outputs road mask)
        if isa(net,'SeriesNetwork') || isa(net,'DAGNetwork')
            lastLayer = net.Layers(end);
            if contains(class(lastLayer),'Regression')
                % Fast encoder-decoder: output is road probability mask
                mask = predict(net, img);
                roadFraction = mean(mask(:) > 0.5);
            else
                % DeepLabv3+: semanticseg output
                predMask = semanticseg(img, net, 'OutputType','uint8');
                roadFraction = mean(predMask(:) == 1);
            end
        else
            roadFraction = 0.5;
        end
    catch
        % Any error → fallback
        H = size(frame,1);
        roadFraction = 0.45 + 0.05*randn();
        roadFraction = max(0.2, min(0.9, roadFraction));
    end
end
