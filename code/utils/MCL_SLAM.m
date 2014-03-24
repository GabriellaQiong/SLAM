% MCL_SLAM script for ESE 650 Learning in Robotics
% Written by Qiong Wang at University of Pennsylvania
% 03/23/2014

%% Initialization
% Map
global MAP
MAP.res   = 0.05;  %meters
MAP.xmin  = -40;   %meters
MAP.ymin  = -40;
MAP.xmax  =  40;
MAP.ymax  =  40;
MAP.sizex = ceil((MAP.xmax - MAP.xmin) / MAP.res + 1); %cells
MAP.sizey = ceil((MAP.ymax - MAP.ymin) / MAP.res + 1);
MAP.map   = zeros(MAP.sizex,MAP.sizey,'int8');

% Constant Params
Tsensor   = trans([0.1 0 0])*rotz(0)*roty(0)*rotx(0);
pose      = [0; 0; 0];
motion    = [dc; alpha];
numP      = 40;               % Number of particles
particles = rand(3, numP);  
w         = zeros(numP, 1); 
X1        = zeros(3, numP);
x         = zeros(3, numP);

%% Looping
for i = 200 : numData
    % Transforms
    Timu  = rotz(rpy(3, i))*roty(rpy(2, i))*rotx(rpy(1, i));
    Tpose = trans([pose(1 : 2) 0]);
    T     = Tpose * Timu * Tsensor;

    % xy position in the sensor frame
    xs0 = (ranges(:,i) .* cos(angles))';
    ys0 = (ranges(:,i) .* sin(angles))';
    
    % convert to body frame using initial transformation
    X = [xs0; ys0; zeros(size(xs0)); ones(size(xs0))];
    Y = T * X;
    
    % transformed xs and ys
    xs1 = Y(1,:);
    ys1 = Y(2,:);
    
    % convert from meters to cells
    xis = ceil((xs1 - MAP.xmin) ./ MAP.res);
    yis = ceil((ys1 - MAP.ymin) ./ MAP.res);
    
    % check the indices and populate the map
    indGood = (xis > 1) & (yis > 1) & (xis < MAP.sizex) & (yis < MAP.sizey);
    inds    = sub2ind(size(MAP.map), xis(indGood), yis(indGood));
    MAP.map(inds) = 100;
    
    % compute correlation
    x_im = MAP.xmin : MAP.res : MAP.xmax; % x-positions of each pixel of the map
    y_im = MAP.ymin : MAP.res : MAP.ymax; % y-positions of each pixel of the map
    
    x_range = -1 : 0.05 : 1;
    y_range = -1 : 0.05 : 1;
    
    c = map_correlation(MAP.map, x_im, y_im, Y(1:3,:), x_range, y_range);
    
    for j = 1 : numP
        x(:, j) = sample_motion(motion(:, i), particles(:, j));
        w(j)    = landmark_model_known_correspondence(x(:, j), z1, c1, MAP.map, sigmaZ);
    end
    
    totalWeight = sum(w(:));
    
    set(ht, 'String', {'Test Map Correlation'; sprintf('Time = %05f s',tsEn(i) - tsEn(1))});
    
    % plot original lidar points
    subplot(1, 3, 1);
    plot(xs1,ys1,'.');
    axis square;
    title('Original LIDAR points');
    drawnow;
    
    % plot map
    subplot(1, 3, 2);
%     imagesc(MAP.map);
    imshow(MAP.map);
%     colormap(winter);
    axis square;
    title('Plot of the map');
    drawnow;
    
    % plot correlation
    subplot(1, 3, 3);
    surf(c);
    title('Plot of correlation');
    axis square;
    drawnow;
end

return;
h1 = figure('NumberTitle', 'off', 'Name', 'Test LIDAR Visualization');
set(h1,'units','normalized','position',[0 .4 .6 .6]);
ht = suptitle({'Test Map Correlation'; sprintf('Time = %05f s',tsEn(1) - tsEn(1))});
