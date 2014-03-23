function [ varargout ] = dead_reckoning(dc, alpha, theta, ts, verbose)
% DEAD_RECKONING() shows the simple odometry info from encoders and imu
% Written by Qiong Wang at University of Pennsylvania
% 03/21/2014

if nargin < 4
    verbose = false;
end

theta = 0;
angle = cumsum(alpha) - alpha / 2;
dx    = dc .* cos(angle);
dy    = dc .* sin(angle);
x     = cumsum(dx) / 1000;
y     = cumsum(dy) / 1000;
theta = cumsum(theta + alpha);

if nargout == 1 || nargout == 0
    varargout{1} = [x; y; theta];
elseif nargout == 3
    varargout{1} = x;
    varargout{2} = y;
    varargout{3} = theta;
end

if ~verbose
    return;
end

ts = ts - ts(1);

figure('NumberTitle', 'off', 'Name', 'Dead Reckoning Data Visualization');
subplot(3, 1, 1);
plot(ts, x,'g','LineWidth',1.2);
xlim([ts(1), ts(end)]); xlabel('t/s'); ylabel('x/mm')
obj1= title('Plot of x position');
set(obj1,'Interpreter','Latex'); clear obj1;
subplot(3, 1, 2);
plot(ts, y,'m','LineWidth',1.2);
xlim([ts(1), ts(end)]); xlabel('t/s'); ylabel('y/mm')
obj1= title('Plot of y position');
set(obj1,'Interpreter','Latex'); clear obj1;
subplot(3, 1, 3);
plot(ts, theta,'b','LineWidth',1.2);
xlim([ts(1), ts(end)]); xlabel('t/s'); ylabel('theta/rad')
obj1= title('Plot of theta angle');
set(obj1,'Interpreter','Latex'); clear obj1;

figure('NumberTitle', 'off', 'Name', 'Dead Reckoning Path Visualization');
plot(x, y, 'LineWidth', 1.2);
end