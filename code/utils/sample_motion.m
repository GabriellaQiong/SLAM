function pose_ = sample_motion(motion, pose)
% SAMPLE_MOTION() samples the motion model by adding random value to 
% Written by Qiong Wang at University of Pennsylvania
% 03/23/2014

coef  = [.01, .01, .01, .01, .001, .001];
s     = motion(1);
alpha = motion(2);

s_     = s + randn*sqrt(coef(1)*(s.^2) + coef(2)*(alpha.^2));
alpha_ = alpha + randn*sqrt(coef(3)*(s.^2)+coef(4)*(alpha.^2));
rem    = randn*sqrt(coef(5)*(s.^2)+coef(6)*(alpha.^2));

pose_    = zeros(3,1);
pose_(1) = pose(1) - (s_/alpha_)*sin(pose(3)) + (s_/alpha_)*sin(pose(3) + alpha_);
pose_(2) = pose(2) + (s_/alpha_)*cos(pose(3)) - (s_/alpha_)*cos(pose(3) + alpha_);
pose_(3) = pose(3) + alpha_ + rem;
pose_(3) = mod(pose_(3) + pi,2*pi)-pi;
end