% MCL_SLAM script for ESE 650 Learning in Robotics
% Written by Qiong Wang at University of Pennsylvania
% 03/23/2014

%% Initialization
% Map
% global MAP
res   = 0.1;  %meters
xmin  = -40;   %meters
ymin  = -40;
xmax  =  40;
ymax  =  40;
sizex = ceil((xmax - xmin) / res + 1); %cells
sizey = ceil((ymax - ymin) / res + 1);
map   = zeros(sizex, sizey,'int8') - 10;

% Constant Params
Tsensor = trans([0.1 0 0]) * rotz(0) * roty(0) * rotx(0);
pose    = [0; 0; 0];
motion  = [dc; alpha];
numP    = 20;               % Number of particles
ps      = zeros(3, numP);
factor  = 0.8;
xim     = xmin : res : xmax; % x-positions of each pixel of the map
yim     = ymin : res : ymax; % y-positions of each pixel of the map
xRange  = -1 : 0.1 : 1;
yRange  = -1 : 0.1 : 1;
w       = ones(numP, 1) / numP; 

% Function handels
m2cell    = @(val, min, reso) ceil((val - min) ./ reso);

% Plot
h1 = figure('NumberTitle', 'off', 'Name', 'MCL Result Visualization');
set(h1,'units','normalized','position',[0 .3 .6 .6]);
% ht = suptitle({'Test Map Correlation'; sprintf('Time = %05f s',tsEn(1) - tsEn(1))});

%% Looping
for i = 300 : numData

    fprintf('Processing %d...\n', i);
    numP = size(ps, 2);

    % Transforms
%     Timu  = rotz(rpy(3, i)) * roty(rpy(2, i)) * rotx(rpy(1, i));
    Timu = rotz(pose(3))*roty(0)*rotx(0);
    Tpose = trans([pose(1 : 2)', 0]);
    Ttmp  = Timu * Tsensor;
    T     = Tpose * Ttmp;

    % Position in the sensor frame
    xs0 = (ranges(:,i) .* cos(angles))';
    ys0 = (ranges(:,i) .* sin(angles))';
    
    % Convert to body frame using initial transformation
    X = [xs0; ys0; zeros(size(xs0)); ones(size(xs0))];
    Y = T * X;
    
    % Transformed xs and ys
    xs1 = Y(1,:);
    ys1 = Y(2,:);
    
    % Convert from meters to cells
    xis  = m2cell(xs1, xmin, res);
    yis  = m2cell(ys1, ymin, res);
    xcis = m2cell(pose(1), xmin, res);
    ycis = m2cell(pose(2), ymin, res);
    
    % Check the indices and populate the map
    indGood       = (xis > 1) & (yis > 1) & (xis < sizex) & (yis < sizey);
    inds          = sub2ind(size(map), xis(indGood), yis(indGood));
    [xx, yy]      = getMapCellsFromRay(xcis, ycis, xis(indGood), yis(indGood));
    effIdx        = (xx > 1) & (yy > 1) & (xx < sizex) & (yy < sizey);
    path          = sub2ind(size(map), xx(effIdx), yy(effIdx));
    map(path)     = max(map(path) - 50, 0);
    map(inds)     = map(inds) + 50;
    
    if i == 1
        continue;
    end
    
    for j = 1 : numP
        ps(:, j) = sample_motion(motion(:, i - 1), pose);
%         ps(:, j) = sample_motion(motion(:, i - 1), ps(:, j));
        Tpose    = trans([ps(1 : 2, j)', 0]);
        T        = Tpose * Ttmp;
        Y        = T * X;        
        c        = map_correlation(map, xim, yim, Y(1:3, :), xRange, yRange);
        w(j)     = sum(c(:));
    end
    
    % Check whether to resample
    wSum = sum(w);
    w    = w / wSum;
    [~, idx] = max(w);
%     pose_ = pose;
    pose = ps(:, idx(1));
%     Neff = 1 / sum(w.^2);
% 
%     if Neff < factor * numP
% %        index = resample(w, numP);
%          w  = ones(numP, 1) / numP;
% %        ps    = ps(:, index);
%          for jjj = 1 : numP
%              ps(:, j) = sample_motion(motion(:, i - 1), pose_); 
%          end
%     end
%     
    
%     set(ht, 'String', {'Test Map Correlation'; sprintf('Time = %05f s',tsEn(i) - tsEn(1))});
    
    % plot original lidar points
    subplot(1, 2, 1);
    plot(xs1,ys1,'.');
    axis square;
    title('Original LIDAR points');

    % plot map
    subplot(1, 2, 2);
    imshow(map);
    %     imagesc(MAP.map);
    %     colormap(winter);
    axis square;
    title('Plot of the map');
    
    % plot correlation
%     subplot(1, 3, 3);
%     surf(c);
%     title('Plot of correlation');
%     axis square;
    drawnow;
end
% axis square;
% imagesc(map);