% MCL_SLAM script for ESE 650 Learning in Robotics
% Written by Qiong Wang at University of Pennsylvania
% 03/23/2014

%% Initialization
% Map
% global MAP
MAP.res   = 1;  %meters
MAP.xmin  = -40;   %meters
MAP.ymin  = -40;
MAP.xmax  =  40;
MAP.ymax  =  40;
MAP.sizex = ceil((MAP.xmax - MAP.xmin) / MAP.res + 1); %cells
MAP.sizey = ceil((MAP.ymax - MAP.ymin) / MAP.res + 1);
MAP.map   = zeros(MAP.sizex,MAP.sizey,'int8') - 10;

% Constant Params
Tsensor = trans([0.1 0 0]) * rotz(0) * roty(0) * rotx(0);
pose    = [0; 0; 0];
motion  = [dc; alpha];
numP    = 10;               % Number of particles
ps      = rand(3, numP);
factor  = 0.5;

% Function handels
m2cell    = @(val, min, res) ceil((val - min) ./ res);

% Plot
h1 = figure('NumberTitle', 'off', 'Name', 'MCL Result Visualization');
% set(h1,'units','normalized','position',[0 .3 .6 .6]);
% ht = suptitle({'Test Map Correlation'; sprintf('Time = %05f s',tsEn(1) - tsEn(1))});

%% Looping
for i = 200 : numData
    numP = size(ps, 2);
    w    = zeros(numP, 1); 

    % Transforms
    Timu  = rotz(rpy(3, i)) * roty(rpy(2, i)) * rotx(rpy(1, i));
    Tpose = trans([pose(1 : 2)', 0]);
    T     = Tpose * Timu * Tsensor;

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
    xis  = m2cell(xs1, MAP.xmin, MAP.res);
    yis  = m2cell(ys1, MAP.ymin, MAP.res);
    xcis = m2cell(pose(1), MAP.xmin, MAP.res);
    ycis = m2cell(pose(2), MAP.xmin, MAP.res);
    
    % Check the indices and populate the map
    indGood       = (xis > 1) & (yis > 1) & (xis < MAP.sizex) & (yis < MAP.sizey);
    inds          = sub2ind(size(MAP.map), xis(indGood), yis(indGood));
    [xx, yy]      = getMapCellsFromRay(xcis, xcis, xis(indGood), yis(indGood));
    effIdx        = (xx > 1) & (yy > 1) & (xx < MAP.sizex) & (yy < MAP.sizey);
    path          = sub2ind(size(MAP.map), xx(effIdx), yy(effIdx));
    MAP.map(path) = max(MAP.map(path) - 50, 0);
    MAP.map(inds) = MAP.map(inds) + 50;
    
    if i == 1
        continue;
    end
    
    % Compute correlation
    xim = MAP.xmin : MAP.res : MAP.xmax; % x-positions of each pixel of the map
    yim = MAP.ymin : MAP.res : MAP.ymax; % y-positions of each pixel of the map
    
    xRange = -1 : 0.05 : 1;
    yRange = -1 : 0.05 : 1;
    
    for j = 1 : numP
        ps(:, j) = sample_motion(motion(:, i), pose);
        Tpose    = trans([ps(1 : 2, j)', 0]);
        T        = Tpose * Timu * Tsensor;
        Y        = T * X;
        c        = map_correlation(MAP.map, xim, yim, Y(1:3, :), xRange, yRange);
        w(j)     = max(c(:));
    end
    
    % Check whether to resample
    wSum = sum(w(:));
    w    = w / wSum;
    Neff = (sum(w(:)))^2 / sum(w.^2);
    while Neff < factor * numP
        index = resample(w, numP);
        w     = w(index);
        ps    = ps(:, index);
    end
    [~, idx] = max(w);
    pose = ps(:, idx(1));
    
%     set(ht, 'String', {'Test Map Correlation'; sprintf('Time = %05f s',tsEn(i) - tsEn(1))});
%     
%     % plot original lidar points
%     subplot(1, 3, 1);
%     plot(xs1,ys1,'.');
%     axis square;
%     title('Original LIDAR points');
%     drawnow;
%     
%     % plot map
%     subplot(1, 3, 2);
%     imshow(MAP.map);
%     %     imagesc(MAP.map);
%     %     colormap(winter);
%     axis square;
%     title('Plot of the map');
%     drawnow;
%     
%     % plot correlation
%     subplot(1, 3, 3);
%     surf(c);
%     title('Plot of correlation');
%     axis square;
%     drawnow;
end
axis square;
imagesc(MAP.map);