function [bboxes, scores, labels, detected] = RunYOLO(frame, detector, threshold)
% RunYOLO - Object detection with 3 paths:
%   1. Custom ResNet CNN patch classifier (built-in, trained on IDD)
%   2. YOLOv4/v2 standard detector
%   3. Colour-heuristic fallback

    if nargin < 3; threshold = 0.65; end   % raised from 0.4 → less false positives

    detected.hasPerson   = false;
    detected.hasCow      = false;
    detected.hasAuto     = false;
    detected.hasBike     = false;
    detected.hasLight    = false;
    detected.closestDist = Inf;

    bboxes = zeros(0,4);
    scores = zeros(0,1);
    labels = categorical({});

    if isempty(detector)
        [bboxes, scores, labels] = FallbackDetect(frame);

    elseif isstruct(detector) && isfield(detector,'type') && ...
           strcmp(detector.type,'resnet18_patch')
        % Custom CNN patch sliding window
        [bboxes, scores, labels] = ResNetSlideDetect(frame, detector, threshold);

    else
        % Standard YOLO detector
        try
            inSize   = detector.Network.Layers(1).InputSize(1:2);
            inputImg = imresize(frame, inSize);
            [bboxes, scores, labels] = detect(detector, inputImg, ...
                'Threshold', threshold, 'SelectStrongest', true);
        catch
            [bboxes, scores, labels] = FallbackDetect(frame);
        end
    end

    if isempty(labels); return; end

    labelStr = string(labels);
    detected.hasPerson = any(strcmp(labelStr,'person'));
    detected.hasCow    = any(strcmp(labelStr,'cow'));
    detected.hasAuto   = any(strcmp(labelStr,'auto_rickshaw'));
    detected.hasBike   = any(strcmp(labelStr,'bicycle'));
    detected.hasLight  = any(strcmp(labelStr,'traffic_light'));

    if ~isempty(bboxes)
        areas     = bboxes(:,3) .* bboxes(:,4);
        frameArea = size(frame,1) * size(frame,2);
        ratios    = areas / frameArea;
        dists     = 30 ./ (ratios * 100 + 1);
        detected.closestDist = min(dists);
    end
end


%% ─── ResNet-18 Sliding Window ────────────────────────────────────────────────
function [bboxes, scores, labels] = ResNetSlideDetect(frame, detector, threshold)
    net        = detector.network;
    classNames = detector.classNames;
    pSz        = detector.patchSize;
    [H, W, ~]  = size(frame);

    bboxes = zeros(0,4); scores = zeros(0,1); lbl = {};

    % Only 2 scales + larger stride = far fewer windows = no over-detection
    scales  = [0.5 0.8];
    for s = scales
        wH = round(H*s); wW = round(W*s);
        if wH < 20 || wW < 20; continue; end
        strideH = max(1, round(wH * 0.6));   % bigger stride = fewer windows
        strideW = max(1, round(wW * 0.6));

        for y = 1:strideH:(H-wH+1)
            for x = 1:strideW:(W-wW+1)
                patch = imresize(frame(y:y+wH-1, x:x+wW-1, :), pSz);
                try
                    [predLbl, predConf] = classify(net, patch);
                    conf    = max(predConf);
                    predLbl = char(predLbl);
                    if conf >= threshold && ~strcmp(predLbl,'background')
                        bboxes(end+1,:) = [x, y, wW, wH];
                        scores(end+1)   = conf;
                        lbl{end+1}      = predLbl;
                    end
                catch; continue; end
            end
        end
    end

    % NMS — scores must be column vector Mx1
    if ~isempty(bboxes)
        [bboxes, scores, idx] = selectStrongestBbox(bboxes, scores(:), ...
            'RatioType','Min','OverlapThreshold',0.35);
        lbl    = lbl(idx);
        labels = categorical(lbl(:));
    else
        labels = categorical({});
        scores = zeros(0,1);
    end
end


%% ─── Colour Fallback ─────────────────────────────────────────────────────────
function [bboxes, scores, labels] = FallbackDetect(frame)
    bboxes = zeros(0,4); scores = zeros(0,1); lbl = {};
    % Orange/yellow blobs → person
    mask_ped = frame(:,:,1)>180 & frame(:,:,2)>130 & frame(:,:,2)<200 & frame(:,:,3)<80;
    if any(mask_ped(:))
        [r,c]=find(mask_ped);
        bboxes(end+1,:)=[min(c),min(r),max(c)-min(c)+1,max(r)-min(r)+1];
        scores(end+1)=0.70; lbl{end+1}='person';
    end
    % Brown → cattle
    mask_cow = frame(:,:,1)>120 & frame(:,:,1)<180 & frame(:,:,2)>80 & frame(:,:,2)<130 & frame(:,:,3)<80;
    if any(mask_cow(:))
        [r,c]=find(mask_cow);
        bboxes(end+1,:)=[min(c),min(r),max(c)-min(c)+1,max(r)-min(r)+1];
        scores(end+1)=0.65; lbl{end+1}='cow';
    end
    if isempty(lbl); labels=categorical({}); else; labels=categorical(lbl); end
end
