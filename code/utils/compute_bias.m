function bias = compute_bias(imuVals)
% COMPUTE_BIAS() computes the static bias of imu values
%
% Written by Qiong Wang at University of Pennsylvania
% 02/06/2014

bias        = mean(imuVals(:, 1 : 150), 2);
Vref        = 3300;
sensitivity = 300;
bias(3)     = - 1/ (Vref / 1023 / sensitivity) + bias(3);
end