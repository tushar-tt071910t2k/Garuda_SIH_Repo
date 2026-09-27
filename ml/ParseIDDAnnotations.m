function [annotations, labelStats] = ParseIDDAnnotations(dataRoot, classNames, maxFiles)
% ParseIDDAnnotations — Parse IDD polygon JSONs into bbox table
% Properly matches leftImg8bit images with gtFine polygon JSONs
% by city folder (e.g. val/67 ↔ leftImg8bit/val/67)
%
% dataRoot   : e.g. 'e:\sih_matlab\ml\data'
% classNames : cell array of target classes
% maxFiles   : max JSON files to process (default: all)

    if nargin < 2
        classNames = {'person','cow','auto_rickshaw','bicycle','car','truck','traffic_light','pushcart'};
    end
    if nargin < 3; maxFiles = Inf; end

    % IDD label → our class name
    labelMap = containers.Map('KeyType','char','ValueType','char');
    labelMap('person')           = 'person';
    labelMap('rider')            = 'person';
    labelMap('pedestrian')       = 'person';
    labelMap('animal')           = 'cow';
    labelMap('cow')              = 'cow';
    labelMap('buffalo')          = 'cow';
    labelMap('dog')              = 'cow';
    labelMap('autorickshaw')     = 'auto_rickshaw';
    labelMap('auto-rickshaw')    = 'auto_rickshaw';
    labelMap('three-wheeled-vehicle') = 'auto_rickshaw';
    labelMap('bicycle')          = 'bicycle';
    labelMap('motorcycle')       = 'bicycle';
    labelMap('two-wheeler')      = 'bicycle';
    labelMap('car')              = 'car';
    labelMap('vehicle')          = 'car';
    labelMap('motor')            = 'car';
    labelMap('truck')            = 'truck';
    labelMap('bus')              = 'truck';
    labelMap('traffic light')    = 'traffic_light';
    labelMap('traffic_light')    = 'traffic_light';
    labelMap('pushcart')         = 'pushcart';
    labelMap('push cart')        = 'pushcart';

    % Ignore classes
    skipLabels = {'road','sidewalk','background','sky','vegetation','terrain',...
                  'building','wall','fence','unlabeled','fallback background',...
                  'guard rail','ego vehicle','drivable area','lane marking'};

    % Build a fast lookup: imageID → full image path
    fprintf('Building image ID lookup map...\n');
    allImgs    = dir(fullfile(dataRoot,'leftImg8bit','**','*_leftImg8bit.png'));
    imgLookup  = containers.Map('KeyType','char','ValueType','char');
    for i = 1:numel(allImgs)
        imgID = strrep(allImgs(i).name,'_leftImg8bit.png','');
        imgLookup(imgID) = fullfile(allImgs(i).folder, allImgs(i).name);
    end
    fprintf('  Image lookup built: %d images indexed\n', imgLookup.Count);

    % Find all JSON files
    allJSONs = dir(fullfile(dataRoot,'gtFine','**','*_gtFine_polygons.json'));
    nFiles   = min(numel(allJSONs), maxFiles);
    fprintf('Processing %d / %d JSON annotation files...\n', nFiles, numel(allJSONs));

    ImageFile  = {};
    BBoxes     = {};
    Labels     = {};
    labelCounts = containers.Map(classNames, zeros(1,numel(classNames)));
    validCount  = 0;

    for i = 1:nFiles
        if mod(i,1000) == 0
            fprintf('  [%d/%d] valid so far: %d\n', i, nFiles, validCount);
        end
        jsonPath = fullfile(allJSONs(i).folder, allJSONs(i).name);

        try
            raw  = fileread(jsonPath);
            data = jsondecode(raw);
            W    = data.imgWidth;
            H    = data.imgHeight;

            % Find matching image
            imgID    = strrep(allJSONs(i).name,'_gtFine_polygons.json','');
            if isKey(imgLookup, imgID)
                imgPath = imgLookup(imgID);
            else
                imgPath = '';   % No real image — will use synthetic
            end

            boxes = zeros(0,4);
            lbls  = {};

            for j = 1:numel(data.objects)
                obj    = data.objects(j);
                rawLbl = lower(strtrim(obj.label));

                % Skip background/road classes
                if any(strcmpi(rawLbl, skipLabels)); continue; end

                % Map to our classes
                if isKey(labelMap, rawLbl)
                    mappedLbl = labelMap(rawLbl);
                else
                    continue;
                end
                if ~ismember(mappedLbl, classNames); continue; end

                % Polygon → bbox
                try
                    poly = obj.polygon;
                    if iscell(poly)
                        pts = cell2mat(poly);
                    else
                        pts = poly;
                    end
                    if isempty(pts) || size(pts,1) < 2; continue; end

                    xs = pts(:,1); ys = pts(:,2);
                    x1 = max(0,   min(xs));
                    y1 = max(0,   min(ys));
                    bw = min(W, max(xs)) - x1;
                    bh = min(H, max(ys)) - y1;
                    if bw < 5 || bh < 5; continue; end

                    boxes(end+1,:) = [x1, y1, bw, bh];  %#ok<AGROW>
                    lbls{end+1}    = mappedLbl;           %#ok<AGROW>
                    if isKey(labelCounts, mappedLbl)
                        labelCounts(mappedLbl) = labelCounts(mappedLbl) + 1;
                    end
                catch; continue; end
            end

            if numel(lbls) > 0
                validCount = validCount + 1;
                ImageFile{validCount}  = imgPath;
                BBoxes{validCount}     = boxes;           % Nx4
                Labels{validCount}     = categorical(lbls(:), classNames);  % MUST be Nx1 column
            end

        catch; continue; end
    end

    annotations = table(ImageFile(:), BBoxes(:), Labels(:), ...
                        'VariableNames', {'ImageFile','BBoxes','Labels'});

    keys_   = classNames(:);
    counts_ = num2cell(cellfun(@(k) labelCounts(k), keys_));
    labelStats = cell2table([keys_, counts_], 'VariableNames',{'Class','Count'});

    fprintf('Done. Valid annotated samples: %d\n', validCount);
    fprintf('Real image paths found: %d / %d\n', ...
        sum(~cellfun(@isempty, annotations.ImageFile)), validCount);
    disp(labelStats);
end
