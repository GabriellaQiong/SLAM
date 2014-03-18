function displayKinectDepth(disparity)
persistent hFig yis zis

if isempty(hFig)
  
  %generate and store arrays of indices
  [zis yis] = meshgrid((1:480),(1:640));
  yis = yis(:)';
  zis = zis(:)';
  
  %create figure
  figure(3), clf(gcf);
  hFig = plot3(0,0,0,'b.');
  axis([0 10 -3 3 -1 2]);
end

%convert from uint16 to double
disp2 = double(disparity(:))';

%get the x values (forward distance)
%slightly different from ROS values
xs = 1.03 ./ (-0.00304 .* disp2 + 3.31);

%some calibration parameters from ROS package
fx = 585.05108211;
fy = 585.05108211;
cx = 315.83800193;
cy = 242.94140713;

%calculate y and z values
ys = -(yis-cx) ./ fx .* xs;
zs = -(zis-cy) ./ fy .* xs;

%transform based on the sensor pitch
T = roty(0.36);
Y = T*[xs;ys;zs;ones(size(xs))];

xs2 = Y(1,:);
ys2 = Y(2,:);
zs2 = Y(3,:);

%extract good values
indGood = xs > 0;

set(hFig,'xdata',xs2(indGood),'ydata', ...
         ys2(indGood),'zdata',zs2(indGood));



