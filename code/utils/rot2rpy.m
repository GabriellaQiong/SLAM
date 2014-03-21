function [ varargout ] = rot2rpy(WRB)
% ROT2RPY() converts rotation matrix to raw pitch yaw angles
%
% Written by Qiong Wang at University of Pennsylvania
% 02/06/2014

% WRB = [cos(yaw)*cos(pitch), 
%        cos(yaw)*sin(roll)*sin(pitch) - cos(roll)*sin(yaw), 
%        sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch);]
%       [cos(pitch)*sin(yaw), 
%        cos(roll)*cos(yaw) + sin(roll)*sin(yaw)*sin(pitch), 
%        cos(roll)*sin(yaw)*sin(pitch) - cos(yaw)*sin(roll);]
%       [-sin(pitch), cos(pitch)*sin(roll), cos(roll)*cos(pitch)];

pitch = asin(-WRB(3,1));
roll  = atan2(WRB(3,2) / cos(pitch), WRB(3,3)/cos(pitch));
yaw   = atan2(WRB(2,1) / cos(pitch), WRB(1,1) / cos(pitch));

if nargout == 1 || nargout == 0
    varargout{1} = [roll; pitch; yaw];
elseif nargout == 3
    varargout{1} = roll;
    varargout{2} = pitch;
    varargout{3} = yaw;
end
end