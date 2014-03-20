% Run Script for SLAM for ESE 650 Learning in Robotics
% Written by Qiong Wang at University of Pennsylvania
% 03/17/2014

%% Clear up
clear all;
close all;
clc;

%% Parameters
% Flags
check   = false;                 % Whether to do sanity check
verbose = false;                 % Whether to show the details

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
imu      = load(fullfile(dataDir, ['imuRaw', num2str(dataIdx)]));

%% Parse data
