function [x, y, theta, dl, dr] = parse_encoders(Encoders)
% Run Script for SLAM for ESE 650 Learning in Robotics
% Written by Qiong Wang at University of Pennsylvania
% 03/20/2014

% Params
res      = 360;                     % counts/ revolution
diameter = 254;                     % mm 
coef     = 2 * pi * diameter / res; % 
width    = 165.1;                   % mm

% Function handle
dCounts = Encoders.counts - [Encoders.counts(:, 1), Encoders.counts(:, 1 : end - 1)];

% Load Data
FR  = dCounts(1, :);
FL  = dCounts(2, :);
RR  = dCounts(3, :);
RL  = dCounts(4, :);
ts  = Encoders.ts;

% Compute distance
dl = (FL + RL) / 2 * coef;
dr = (FR + RR) / 2 * coef;
dc = (dl + lr) / 2;

% Compute rotation angle (Right hand rule)
alpha = (dr - dl) / width;


end