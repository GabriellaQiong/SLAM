function [dc, alpha, ts] = parse_encoders(Encoders)
% PARSE_ENCODERS() parses the raw data from encoders
% Written by Qiong Wang at University of Pennsylvania
% 03/20/2014

% Params
res      = 360;                             % counts/ revolution
diameter = 254;                             % mm 
coef     = pi * diameter / res;
scale    = 1 / 1.85;                             % scale for slipping
width    = (476.25 -311.15) / 2 + 311.15;   % mm

% Load Data
FR  = Encoders.counts(1, :);
FL  = Encoders.counts(2, :);
RR  = Encoders.counts(3, :);
RL  = Encoders.counts(4, :);
ts  = Encoders.ts;

% Compute distance
dl = (FL + RL) / 2 * coef;
dr = (FR + RR) / 2 * coef;
dc = (dl + dr) / 2 / 1000;

% Compute rotation angle (Right hand rule)
alpha = (dr - dl) / width * scale;
end