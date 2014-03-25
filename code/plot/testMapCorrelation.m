%% Mapping
% global MAP

MAP1.res   = 1;  %meters

MAP1.xmin  = -40;  %meters
MAP1.ymin  = -40;
MAP1.xmax  =  40;
MAP1.ymax  =  40;


% dimensions of the map
MAP1.sizex  = ceil((MAP1.xmax - MAP1.xmin) / MAP1.res + 1); %cells
MAP1.sizey  = ceil((MAP1.ymax - MAP1.ymin) / MAP1.res + 1);

MAP1.map = zeros(MAP1.sizex,MAP1.sizey,'int8');

% assuming initial pose of x=0,y=0,yaw=0, put the first scan into the map
% also, assume that roll and pitch are 0 (not true in general - use IMU!)

% % make the origin of the robot's frame at its geometrical center
h1 = figure('NumberTitle', 'off', 'Name', 'Test LIDAR Visualization');
% set(h1,'units','normalized','position',[0 .3 .6 .6]);
% ht = suptitle({'Test Map Correlation'; sprintf('Time = %05f s',tsEn(1) - tsEn(1))});

% Function handels
m2cell    = @(val, min, res) ceil((val - min) ./ res);

%% Looping
for i = 200 : numData
    % sensor to body transform
    Tsensor = trans([0.1 0 0])*rotz(0)*roty(0)*rotx(0);
    
    % transform for the imu reading (assuming zero for this example)
    Timu = rotz(rpy(3, i))*roty(rpy(2, i))*rotx(rpy(1, i));
    
    % body to world transform (initially, one can assume it's zero)
    Tpose   = trans([x(i) y(i) 0]);
    
    % full transform from lidar frame to world frame
    T = Tpose*Timu*Tsensor;
    
    % xy position in the sensor frame
    xs0 = (Hokuyo.ranges(:,i).*cos(Hokuyo.angles))';
    ys0 = (Hokuyo.ranges(:,i).*sin(Hokuyo.angles))';
    
    % convert to body frame using initial transformation
    X = [xs0; ys0; zeros(size(xs0)); ones(size(xs0))];
    Y = T*X;
    
    % transformed xs and ys
    xs1 = Y(1,:);
    ys1 = Y(2,:);
    
    % convert from meters to cells
    xis = m2cell(xs1, MAP1.xmin, MAP1.res);  % ceil((xs1 - MAP.xmin) ./ MAP.res);
    yis = m2cell(ys1, MAP1.ymin, MAP1.res);  % ceil((ys1 - MAP.ymin) ./ MAP.res);
    
    xSub = m2cell(x(i), MAP1.xmin, MAP1.res);
    ySub = m2cell(y(i), MAP1.ymin, MAP1.res);
    
    % check the indices and populate the map
    indGood = (xis > 1) & (yis > 1) & (xis < MAP1.sizex) & (yis < MAP1.sizey);
    inds    = sub2ind(size(MAP1.map),xis(indGood),yis(indGood));
    [xPath, yPath] = getMapCellsFromRay(xSub, ySub, xis(indGood), yis(indGood));
    effect  = (xPath > 1) & (yPath > 1) & (xPath < MAP1.sizex) & (yPath < MAP1.sizey);
    path    = sub2ind(size(MAP1.map), xPath(effect), yPath(effect));
    MAP1.map(path) = max(MAP1.map(path) - 50, 0);
    MAP1.map(inds) = MAP1.map(inds) + 50;
    
    % compute correlation
    xim = MAP1.xmin:MAP1.res:MAP1.xmax; %x-positions of each pixel of the map
    yim = MAP1.ymin:MAP1.res:MAP1.ymax; %y-positions of each pixel of the map
    
    xRange = -1:0.05:1;
    yRange = -1:0.05:1;
    
    c = map_correlation(MAP1.map,xim,yim,Y(1:3,:),xRange,yRange);
    
%     set(ht, 'String', {'Test Map Correlation'; sprintf('Time = %05f s',tsEn(i) - tsEn(1))});
%     
% %     plot original lidar points
%     subplot(1, 3, 1);
%     plot(xs1,ys1,'.');
%     axis square;
%     title('Original LIDAR points');
%     drawnow;
%     
%     % plot map
%     subplot(1, 3, 2);
% %     imagesc(MAP.map);
%     imshow(MAP1.map);
% %     hold on;
% %     plot(xSub, ySub, 'MarkerSize', 100, 'MarkerFaceColor','g');
% %     quiver(xSub, ySub, cos(alpha(i)), sin(alpha(i)));
% %     colormap(winter);
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
imagesc(MAP1.map);