function [tilt, rpy, ts] = parse_imu(Imu, verbose)
% PARSE_ACC() parse the raw data from gyrometer 
% Written by Qiong Wang at University of Pennsylvania
% 02/05/2014

if nargin < 2
    verbose = false;
end

% Compute bias
bias = compute_bias(Imu.vals);
ts   = Imu.ts;

% Raw data
accRaw   = Imu.vals(1 : 3, :);
gyroRaw  = Imu.vals(4 : 6, :);

% Parameters
scaleGyro   = 0.0125;
scaleAcc    = 3300 / 1023 / 300;
datNum      = size(gyroRaw, 2);

% Compute accelerometer
accNew = scaleAcc .* bsxfun(@minus, accRaw, bias(1 : 3, :));
accNew = bsxfun(@times, accNew, [-1; -1; 1]);
accNew = bsxfun(@rdivide, accNew, sqrt(sum(accNew .* accNew, 1)));

% Compute tilt
tilt = atan2(sqrt(accNew(1, :).^2 + accNew(2, :).^2), accNew(3, :));
tilt = 0.05 * tilt + 0.95 * [tilt(1) tilt(1 : end - 1)];

% Compute gyro
gyroNew   = scaleGyro .* bsxfun(@minus, gyroRaw, bias(4 : 6, :));
factorMat = [0 1 0; 0 0 1; - 1 0 0];
gyroNew   = factorMat * gyroNew;

% Compute the rotation matrix using quaternion
velNorm = sqrt(sum(gyroNew.^2, 1));
deltaT  = ts - [ts(1), ts(1 : end - 1)];
angle   = velNorm .* deltaT;
axis    = bsxfun(@rdivide, gyroNew, velNorm); 
qDelta  = [cos(angle / 2)' transpose(bsxfun(@times, axis, sin(angle / 2)))];
q       = [1 0 0 0];
rpy     = zeros(3, datNum);

% Note: Gyroscope outputs the rotation according to body frame the rotation
%       should be incremented
for i = 1 : datNum
    q      = quatnormalize(quatmultiply(q, qDelta(i, :)));
    rpy(:, i) = wrapToPi(rot2rpy(quat2dcm(q)));
end

if ~verbose
    return;
end

figure('NumberTitle', 'off', 'Name', 'Imu Data Visualization');
subplot(2, 1, 1);
plot(ts  - Imu.ts(1), tilt,'g','LineWidth',1.2);
xlim([ts(1) - Imu.ts(1), ts(end) - Imu.ts(1)]); xlabel('t/s'); ylabel('tilting/rad')
obj1= title('Plot of tilting angle');
set(obj1,'Interpreter','Latex'); clear obj1;
subplot(2, 1, 2);
plot(ts - Imu.ts(1), rpy(3, :),'m','LineWidth',1.2);
xlim([ts(1) - Imu.ts(1), ts(end) - Imu.ts(1)]); xlabel('t/s'); ylabel('yaw/rad')
obj1= title('Plot of yaw angle');
set(obj1,'Interpreter','Latex'); clear obj1;

end