function annotations = ParseJSONAnnotations(jsonPath)
% ParseJSONAnnotations - Parse COCO-style JSON annotation file into MATLAB table
%
% Input:
%   jsonPath - path to a COCO-format JSON annotation file
%
% Output:
%   annotations - table with columns: ImageFile, BBoxes (Nx4), Labels (Nx1 categorical)
%
% COCO JSON format expected:
%   {"images":[{"id":1,"file_name":"img.jpg"},...],
%    "annotations":[{"image_id":1,"bbox":[x,y,w,h],"category_id":1},...],
%    "categories":[{"id":1,"name":"person"},...]
%   }

    fprintf('Parsing JSON: %s\n', jsonPath);

    raw     = fileread(jsonPath);
    data    = jsondecode(raw);

    % Build category ID → name map
    catMap  = containers.Map('KeyType','int32','ValueType','char');
    for k = 1:numel(data.categories)
        catMap(int32(data.categories(k).id)) = data.categories(k).name;
    end

    % Build image ID → filename map
    imgMap  = containers.Map('KeyType','int32','ValueType','char');
    imgDir  = fileparts(jsonPath);
    for k = 1:numel(data.images)
        imgMap(int32(data.images(k).id)) = fullfile(imgDir, data.images(k).file_name);
    end

    % Group annotations by image
    imageIDs    = arrayfun(@(a) int32(a.image_id), data.annotations);
    uniqueImgIDs = unique(imageIDs);

    ImageFile   = cell(numel(uniqueImgIDs), 1);
    BBoxes      = cell(numel(uniqueImgIDs), 1);
    Labels      = cell(numel(uniqueImgIDs), 1);

    for i = 1:numel(uniqueImgIDs)
        imgID = uniqueImgIDs(i);
        mask  = imageIDs == imgID;
        anns  = data.annotations(mask);

        boxes  = zeros(numel(anns), 4);
        lbls   = cell(numel(anns), 1);

        for j = 1:numel(anns)
            bb        = anns(j).bbox;        % [x y w h]
            boxes(j,:)= [bb(1), bb(2), bb(3), bb(4)];
            catID     = int32(anns(j).category_id);
            if isKey(catMap, catID)
                lbls{j} = catMap(catID);
            else
                lbls{j} = 'unknown';
            end
        end

        if isKey(imgMap, imgID)
            ImageFile{i} = imgMap(imgID);
        else
            ImageFile{i} = sprintf('image_%d.jpg', imgID);
        end
        BBoxes{i} = boxes;
        Labels{i} = categorical(lbls);
    end

    annotations = table(ImageFile, BBoxes, Labels);
    fprintf('  Parsed %d images with %d total annotations.\n', ...
            height(annotations), numel(data.annotations));
end
