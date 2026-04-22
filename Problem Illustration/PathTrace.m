%% V2

%v = VideoReader('WIN_20260306_15_51_57_Pro.mp4');
%v = VideoReader('WIN_20260306_15_06_52_Pro.mp4');
v = VideoReader('IMG_3817 1.mov');
totalFrames = floor(v.Duration * v.FrameRate);
startFrame = 100;      % change as needed
endFrame = totalFrames - 600;  % e.g. set to 500 to only process first 500 frames


% --- Step 1: Background ---
sampleIdx = startFrame:10:endFrame;
firstFrame = read(v, 1);
[h, w, c] = size(firstFrame);
stack = zeros(h, w, c, numel(sampleIdx), 'uint8');
for k = 1:numel(sampleIdx)
    stack(:,:,:,k) = read(v, sampleIdx(k));
end
background = single(median(stack, 4)) / 255;

%% --- Step 2: Composite ---
composite = background;
Threshold = 0.06;

% Define boundaries
end1 = min(startFrame+86, endFrame);
end2 = min(startFrame+140, endFrame);
end3 = min(startFrame+190, endFrame);

% Define frame ranges
firstPart  = startFrame   : 2  : end1;
secondPart = end1        : 80 : end2;
thirdPart  = end2        : 4 : end3;
fourthPart  = end3        : 30 : endFrame;

% Combine indices
sampleIdx = [firstPart, secondPart, thirdPart, fourthPart];


for k = 1:numel(sampleIdx)
    frame = single(read(v, sampleIdx(k))) / 255;
    diff = abs(frame - background);
    mask = max(diff, [], 3) > Threshold;
    mask3 = repmat(mask, 1, 1, 3);
    composite(mask3) = frame(mask3);
end

imwrite(composite, 'new_trail.png')

%% --- Step 2: Composite ---
composite = background;
Threshold = 0.06;

% Define boundaries
end1 = min(startFrame+86, endFrame);
end2 = min(startFrame+140, endFrame);
end3 = min(startFrame+190, endFrame);

% Define frame ranges
firstPart  = startFrame : 2  : end1;
secondPart = end1       : 80 : end2;
thirdPart  = end2       : 4  : end3;
fourthPart = end3       : 30 : endFrame;

% Combine indices
sampleIdx = [firstPart, secondPart, thirdPart, fourthPart];

for k = 1:numel(sampleIdx)
    idx = sampleIdx(k);
    frame = single(read(v, idx)) / 255;

    % Apply slight blur only for fourth part
    if ismember(idx, fourthPart)
        frame = imgaussfilt(frame, 0.8); % small sigma = subtle blur
    end

    diff = abs(frame - background);
    mask = max(diff, [], 3) > Threshold;
    mask3 = repmat(mask, 1, 1, 3);
    composite(mask3) = frame(mask3);
end

imwrite(composite, 'new_trail.png')