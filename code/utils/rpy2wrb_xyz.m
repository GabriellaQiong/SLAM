function [ wRb ] = rpy2wrb_xyz( varargin )
%RPY2WRB_XYZ Converts varargin angles to a rotation matrix from body to world
if nargin == 1
    phi   = varargin{1}(1);
    theta = varargin{1}(2);
    psi   = varargin{1}(3);
elseif nargin == 3;
    phi   = varargin{1};
    theta = varargin{2};
    psi   = varargin{3};
end

Rz = [cos(psi), -sin(psi), 0;
      sin(psi), cos(psi), 0;
      0, 0, 1];
Ry = [cos(theta), 0, sin(theta);
      0, 1, 0;
      -sin(theta), 0, cos(theta)];
Rx = [1, 0, 0;
      0, cos(phi), -sin(phi);
      0, sin(phi), cos(phi)];

wRb = Rz * Ry * Rx;

% wRb = [cos(psi)*cos(theta), ...
%        cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi), ...
%        sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta);
%        cos(theta)*sin(psi), ...
%        cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta), ...
%        cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi);
%        -sin(theta), ...
%        cos(theta)*sin(phi), ...
%        cos(phi)*cos(theta)];

end