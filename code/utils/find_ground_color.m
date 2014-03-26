function color = find_ground_color(depth, rgb, verbose)
% FIND_GROUND_COLOR for kinect for ESE 650 Learning in Robotics
% Written by Qiong Wang at University of Pennsylvania
% 03/24/2014

if nargin < 3
    verbose = false;
end

% Preprocessing format
depth = double(depth);
% rgb   = rgb;

% Compute the surface normal
[nr, nc, ~]  = size(depth);
[x , y]      = meshgrid(1 : nc, 1 : nr);
[nx, ny, nz] = surfnorm(x, y, depth);
surfaceNorm  = [nx(:), ny(:), nz(:)];
[idx, ~]     = kmeans(surfaceNorm, 2);
gx     = x(idx==1);
gy     = y(idx==1);

X = cat(3, nx, ny, nz);
figure;
imshow(X);

A         = zeros(nr,  nc);
A(idx==1) = 1;
bw        = A;
[B, L]    = bwboundaries(bw,'noholes');
stats     = regionprops(L,'Centroid','Area','BoundingBox','Perimeter','Orientation');
blobNum   = length(B);
area      = zeros(blobNum, 1);
center    = zeros(blobNum, 2);
bound     = zeros(blobNum, 4);
theta     = zeros(blobNum, 1);
ratioWH   = zeros(blobNum, 1);
box       = zeros(blobNum, 1);
ratioBox  = zeros(blobNum, 1);

for i = 1 : blobNum
    area(i)      = stats(i).Area;
    center(i, :) = stats(i).Centroid;
    bound(i, :)  = stats(i).BoundingBox;
    theta(i)     = stats(i).Orientation;
    ratioWH(i)   = max(bound(i, 3),bound(i, 4))/min(bound(i, 3),bound(i, 4));
    box(i)       = bound(i, 3)*bound(i, 4);
    ratioBox(i)  = area(i)/box(i);
end

[~, ind] = max(area);
plotBound(bound(ind, :), 'c', theta(i));



figure;
imshow(rgb);
hold on;
plot(gx, gy, 'r.');
return;
idx          = sqrt(sn - []).^2 < thresh;

% Find the pixels for grounds
gx     = x(idx);
gy     = y(idx);

if ~verbose
    return;
end

X = cat(3, nx, ny, nz);
figure;
imshow(X);

subplot(1, 2, 2);
imshow(rgb);
hold on;
plot(gx, gy, 'r.');

% ground = rgb(gx, gy, :);
% color  = median(reshape(ground, [], 3));
color = 1;
end

function [H, inlier_ind, continue_flag] = ransac_plane(x, y, d, thresh)

% Initialize
if nargin < 4, thresh = 5; end
numPts       = numel(x);
ind           = 1 : num_pts;
inlier_ind    = [];
continue_flag = 0;

% Parameters
percent_inlier = 0.99;  % stop when x% of the points are inlier
iter = 300;             % ransac max iteration


% Check the num_pts more than 5
if num_pts < 4
   warning('The descriptors are not enough for TPS RANSAC, please check : )'); 
   continue_flag = 1;
   return;
end

% RANSAC
for i = 1:iter
    perm = randperm(num_pts);
    rand_ind = perm(1:4);
    % estimate homography from source(2) to destination(1)
    Hest = est_homography(x1(rand_ind), y1(rand_ind), x2(rand_ind), y2(rand_ind));
    % apply homography on source(2) to corresponding points in destination(1)
    [x1_est, y1_est] = apply_homography(Hest, x2, y2);
    dist = (x1_est - x1).^2 + (y1_est - y1).^2;
    inlier = ind(dist < thresh^2);
    num_inlier = length(inlier);

    if num_inlier > (num_pts * percent_inlier)
        % break if n% of the points are inlier
        inlier_ind = inlier;
        H = est_homography(x1(inlier_ind), y1(inlier_ind), x2(inlier_ind), y2(inlier_ind));
        break
    elseif num_inlier > length(inlier_ind)
        % update best inlier
        inlier_ind = inlier;
    end
end
H = est_homography(x1(inlier_ind), y1(inlier_ind), x2(inlier_ind), y2(inlier_ind));

end

function [vargout] = plotBound(bound, color, theta)
% PLOTBOUND() plot the bounding box with an orientation angle theta

if nargin < 3
    theta = 0;
else
    theta  =  pi / 2 - theta * pi / 180;
end
    rotMat = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    xCoord = [-bound(3)/2, bound(3)/2, bound(3)/2, -bound(3)/2];
    yCoord = [-bound(4)/2, -bound(4)/2, bound(4)/2, bound(4)/2];
    box    = [xCoord; yCoord];
    rotBox = bsxfun(@plus, rotMat * box, [bound(1) + bound(3)/2; bound(2) + bound(4)/2]);
    rotBox = [rotBox, rotBox(:, 1)];
    plot(rotBox(1, :), rotBox(2, :),[color '-'],'LineWidth',2);
end