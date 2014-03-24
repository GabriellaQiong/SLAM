function theta = angle_fuse(alpha, yaw, t, verbose)
% ANGLE_FUSE() fuses the steering angle from both the imu and encoders
% Written by Qiong Wang at University of Pennsylvania
% 03/20/2014

if nargin < 5
    verbose = false;
end

alphaCum = wrapToPi(cumsum(alpha));
assert(length(yaw) == length(alphaCum), 'The steering angle synced should be the same length!');
theta    = yaw;

if ~verbose
    return;
end

t = t - t(1);

figure('NumberTitle', 'off', 'Name', 'Fuse steering angle data');
plot(t, (alphaCum),'r','LineWidth',1.2); hold on; 
plot(t, (yaw), 'g','LineWidth',1.2); 
xlim([t(1), t(end)]); xlabel('t/s'); ylabel('angle/rad')
obj1= title('Plot of steering angle');obj2 = legend('$Wheel$','$Imu$'); 
set(obj1,'Interpreter','Latex'); set(obj2,'Interpreter','Latex'); clear obj1 obj2;
end