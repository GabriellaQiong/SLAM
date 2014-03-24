% Run Script for SLAM for ESE 650 Learning in Robotics
% Written by Qiong Wang at University of Pennsylvania
% 03/17/2014

%% Clear up
clear all;
close all;
clc;

%% Parameters
% Flags
check   = true;                  % Whether to do sanity check
verbose = true;                  % Whether to show the details

% Predefined values

% Params

%% Paths
scriptDir  = fileparts(mfilename('fullpath'));
dataDir    = '/home/qiong/ese650_data/project4';
outputDir  = fullfile(scriptDir, '../results');
if ~exist(outputDir, 'dir')
    mkdir(outputDir); 
end
addpath(genpath(scriptDir));

%% Load data
dataIdx  = input('Please choose one dataset (20~24): ');
load( fullfile(dataDir, ['Encoders', num2str(dataIdx)]) );
load( fullfile(dataDir, ['Hokuyo', num2str(dataIdx)]), 'Hokuyo0' );
Hokuyo   = Hokuyo0; clear Hokuyo0;
Imu      = load(fullfile(dataDir, ['imuRaw', num2str(dataIdx)]));

%% Parse data
[dc, alpha, tsEn]  = parse_encoders(Encoders);
[tilt, rpy, tsImu] = parse_imu(Imu, verbose);
indices            = sync_time(tsImu, tsEn);
rpy                = rpy(:, indices);
theta              = angle_fuse(alpha, rpy(3, :), tsEn, verbose);

%% Dead Reckoning
% Initialize
[x, y, theta] = dead_reckoning(dc, alpha, theta, tsEn, verbose);
return;
% Sync the time
tsHo   = Hokuyo.ts;
ranges = Hokuyo.ranges;
angles = Hokuyo.angles;

if length(tsEn) > length(tsHo)
    ts  = tsHo;
    ind = sync(tsEn, ts);
    x   = x(ind);
    y   = y(ind);
    rpy = rpy(ind);
else
    ts     = tsEn;
    ind    = sync(tsHo, ts);
    ranges = ranges(ind);
    angles = angles(ind);
end
numData = length(ts);

% Test the results
if check
    testMapCorrelation
end

%% SLAM
