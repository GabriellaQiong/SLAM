function R = rpy2rot(roll, pitch, yaw)
% RPY2ROT() converts raw pitch yaw angles to rotation matrix
%
% Written by Qiong Wang at University of Pennsylvania
% 02/06/2014

R = [ cos(yaw).*cos(pitch) - sin(roll).*sin(yaw).*sin(pitch),... 
      cos(pitch).*sin(yaw) + cos(yaw).*sin(roll).*sin(pitch), -cos(roll).*sin(pitch);...
      
      -cos(roll).*sin(yaw),     cos(roll).*cos(yaw),             sin(roll);
      
      cos(yaw).*sin(pitch) + cos(pitch).*sin(roll).*sin(yaw),...
      sin(yaw).*sin(pitch) - cos(yaw).*cos(pitch).*sin(roll),...
      cos(roll).*cos(pitch)
    ];