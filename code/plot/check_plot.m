% Check Plot Script for Project 2 Orientation Tracking, ESE 650
% Written by Qiong Wang at University of Pennsylvania
% 02/07/2014

%% Synchronize time and data length
offset = sync_time(tsImu, tsVicon);
endIdx = min(size(Racc, 3), size(rots, 3));
if offset < 0
    RaccNew = Racc(:, :, 1 : endIdx + offset + 1);
    RvelNew = Rvel(:, :, 1 : endIdx + offset + 1);
    rotsNew = rots(:, :, -offset : endIdx);
    t       = tsImu(1 : endIdx + offset + 1) - tsImu(1);
elseif offset > 0
    RaccNew = Racc(:, :, offset : endIdx);
    RvelNew = Rvel(:, :, offset : endIdx);
    rotsNew = rots(:, :, 1 : endIdx - offset + 1);
    t       = tsVicon(1 : endIdx - offset + 1) - tsVicon(1);
end

assert(size(RaccNew, 3) == size(rotsNew, 3), 'Dimension error, please check the last step : )');
datNum = length(t);

%% Plot out rotation according to time
if verbose
    h1 = figure(1);
    set(gcf,'units','normalized','position',[0 .4 .6 .6]);
    ht = suptitle({'Sanity Check for Data Conversion'; sprintf('Time = %05f s',t(1))});
else
    ht = [];
end
rpyVicon = zeros(datNum, 3);
rpyAcc   = zeros(datNum, 3);
rpyVel   = zeros(datNum, 3);

for i = 1 : datNum
    subplot(1, 3, 1);
    rotplot(rotsNew(:, :, i), 'Ground Truth', verbose);
    rpyVicon(i, :) = rot2rpy(rotsNew(:, :, i));
    subplot(1, 3, 2);
    rotplot(RaccNew(:, :, i), 'Accelerometer Value', verbose);
    rpyAcc(i, :)   = rot2rpy(RaccNew(:, :, i));
    subplot(1, 3, 3);
    rotplot(RvelNew(:, :, i), 'Gyrometer Value', verbose);
    rpyVel(i, :)   = rot2rpy(RvelNew(:, :, i));
    set(ht, 'String', {'Sanity Check for Data Conversion'; sprintf('Time = %05f s',t(i))});
end
close(gcf);

%% Plot roll pitch yaw angles
h2 = figure(2);
subplot(3,1,1); 
plot(t, rpyVicon(:, 1),'r','LineWidth',1.2); hold on; 
plot(t, rpyAcc(:, 1),'g','LineWidth',1.2); plot(t,rpyVel(:, 1),'b','LineWidth', 1.2);
xlim([t(1), t(end)]); xlabel('t/s'); ylabel('roll/rad')
obj1= title('Plot of roll-angle');obj2 = legend('$Vicon$','$Acc$', '$Gyro$'); 
set(obj1,'Interpreter','Latex'); set(obj2,'Interpreter','Latex'); clear obj1 obj2;

subplot(3,1,2); 
plot(t, rpyVicon(:, 2),'r','LineWidth',1.2); hold on; 
plot(t,rpyAcc(:, 2),'g','LineWidth',1.2); plot(t,rpyVel(:, 2),'b','LineWidth', 1.2);
xlim([t(1), t(end)]); xlabel('t/s'); ylabel('pitch/rad');
obj1= title('Plot of pitch-angle'); obj2 = legend('$Vicon$','$Acc$', '$Gyro$'); 
set(obj1,'Interpreter','Latex'); set(obj2,'Interpreter','Latex'); clear obj1 obj2;

subplot(3,1,3); 
plot(t, rpyVicon(:, 3),'r','LineWidth',1.2); hold on;
plot(t,rpyAcc(:, 3),'g','LineWidth',1.2); plot(t,rpyVel(:, 3),'b','LineWidth', 1.2);
xlim([t(1), t(end)]); xlabel('t/s'); ylabel('yaw/rad');
obj1= title('Plot of yaw-angle'); obj2 = legend('$Vicon$','$Acc$', '$Gyro$'); 
set(obj1,'Interpreter','Latex'); set(obj2,'Interpreter','Latex'); clear obj1 obj2;

fig_save(h2, fullfile(outputDir, 'rpy_comp'), 'pdf');