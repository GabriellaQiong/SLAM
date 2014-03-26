function pose_ = sample_motion(motion, pose)
% SAMPLE_MOTION() samples the motion model by adding random value to 
% Written by Qiong Wang at University of Pennsylvania
% 03/23/2014

s      = motion(1);
alpha  = motion(2);

% s_     = s     + rand * sqrt(s.^2 + alpha.^2) / 10;
% alpha_ = alpha + rand * sqrt(s.^2 + alpha.^2) / 10;
% rem    =         rand * sqrt(s.^2 + alpha.^2) / sqrt(1e3);
a1 = 0.1;
a2 = 0.1;
a3 = 0.05;
a4 = 0.05;
s_     = s     + normrnd(0, abs(a1 * alpha + a2 * s));
alpha_ = alpha + normrnd(0, abs(a4 * alpha + a3 * s));

pose_    = zeros(3,1);
pose_(1) = pose(1) + s_ .* cos(pose(3) + alpha_ / 2);
pose_(2) = pose(2) + s_ .* sin(pose(3) + alpha_ / 2);
pose_(3) = pose(3) + alpha_;
end