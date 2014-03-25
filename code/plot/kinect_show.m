% k = load(fullfile(dataDir, 'kinect20.mat'));
% figure; plot(1:100, k.rgb_ts, 1:100, k.depth_ts) % time series
% figure; imshow(squeeze(k.rgb(1,:,:,:))/255) % first RGB frame
% figure; imshow(squeeze(k.depth(1,:,:))/2048) % first depth frame

% x = load(fullfile(dataDir, 'kinect20.mat'));
f = fopen('temp.jpg', 'w');
fwrite(f, x.zdepth{1});
fclose(f);
imshow('temp.jpg');