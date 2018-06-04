%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%> @file view_data.m  
%>
%> @brief Script for plotting the data from the zero-velocity aided inertial
%> navigations system.
%>
%> @authors Isaac Skog, John-Olof Nilsson
%> @copyright Copyright (c) 2011 OpenShoe, ISC License (open source)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global simdata;


% Close all windows
close all

% Generate a vector with the time scale
N=length(cov);
t=0:simdata.Ts:(N-1)*simdata.Ts;


%% Plot the IMU data
figure(1)
clf
subplot(4,1,1)
plot(t(1:length(t)),u(1:3,:)')
xlabel('time [s]')
ylabel('Specific force [m/s^2]')
title('Specific force (accelerometer) measurements of Right Foot')
legend('x-axis','y-axis','z-axis')
box on
grid on

subplot(4,1,2)
%plot(t,u(4:6,:)'*180/pi)
plot(t(1:length(t)),u(4:6,:)'*180/pi)
xlabel('time [s]')
ylabel('Angular rate  [deg/s]')
title('Angular rate measurements of Right Foot')
legend('x-axis','y-axis','z-axis')
box on
grid on

subplot(4,1,3)
plot(t(1:length(t)),u(7:9,:)')
xlabel('time [s]')
ylabel('Specific force [m/s^2]')
title('Specific force (accelerometer) measurements of Left Foot')
legend('x-axis','y-axis','z-axis')
box on
grid on

subplot(4,1,4)
%plot(t,u(4:6,:)'*180/pi)
plot(t(1:length(t)),u(10:12,:)'*180/pi)
xlabel('time [s]')
ylabel('Angular rate  [deg/s]')
title('Angular rate measurements of Left Foot')
legend('x-axis','y-axis','z-axis')
box on
grid on


%% Plot the trajectory in the horizontal plane
figure(2)
clf
plot(x_h(1,:),x_h(2,:),'b')
hold on
plot(x_h(10,:),x_h(11,:),'r')
hold on
plot(x_h(1,1),x_h(2,1),'gs')
title('Trajectory')
legend('Right Foot Trajectory','Left Foot Trajectory','Start point')
xlabel('x [m]')
ylabel('y [m]')
axis equal
grid on
box on


%% Plot the height profile, the speed and when ZUPTs were applied

figure(3)
clf
subplot(3,1,1)
plot(t,-x_h(3,:),'b')
hold on
plot(t,-x_h(12,:),'r')
title('Height')
legend('Height Right Foot','Height Left Foot')
xlabel('time [s]')
ylabel('z [m]')
grid on
box on

subplot(3,1,2)
plot(t,sqrt(sum(x_h(4:6,:).^2)),'b')
hold on
plot(t,sqrt(sum(x_h(13:15,:).^2)),'r')
title('Speed')
legend('Speed Right Foot','Speed Left Foot')
xlabel('time [s]')
ylabel('|v| [m/s]')
grid on
box on

subplot(3,1,3)
stem(t(1:length(t)),zupt(1,:),'b')
hold on
stem(t(1:length(t)),zupt(2,:),'r')
title('Zupt applied')
legend('Zupt Right Foot','Zupt Left Foot')
xlabel('time [s]')
ylabel('on/off')
grid on
box on


%% Plot the attitude

figure(4)
subplot(2,1,1)
plot(t,unwrap(x_h(7:9,:)')*180/pi)
title('Attitude Right Foot')
xlabel('time [s]')
ylabel('Angle [deg]')
legend('Roll','Pitch','Yaw')
grid on
box on

subplot(2,1,2)
plot(t,unwrap(x_h(16:18,:)')*180/pi) 
title('Attitude Left Foot')
xlabel('time [s]')
ylabel('Angle [deg]')
legend('Roll','Pitch','Yaw')
grid on
box on

%% Plot the diagonal elements of the filter covariance matrices as a
%% function of time

figure(5)
clf
subplot(3,1,1)
plot(t,sqrt(cov(1:3,:))')
title('Position covariance Right Foot')
ylabel('sqrt(cov) [m]')
xlabel('time [s]')
legend('x-axis', 'y-axis','z-axis')
grid on
box on


subplot(3,1,2)
plot(t,sqrt(cov(4:6,:))')
title('Velocity covariance Right Foot')
ylabel('sqrt(cov) [m/s]')
xlabel('time [s]')
legend('x-axis', 'y-axis','z-axis')
grid on
box on

subplot(3,1,3)
plot(t,sqrt(cov(7:9,:))'*180/pi)
title('Heading covariance Right Foot')
ylabel('sqrt(cov) [deg]')
xlabel('time [s]')
legend('Roll', 'Pitch','Yaw')
grid on
box on


figure(6)
clf
subplot(3,1,1)
plot(t,sqrt(cov(10:12,:))')
title('Position covariance Left Foot')
ylabel('sqrt(cov) [m]')
xlabel('time [s]')
legend('x-axis', 'y-axis','z-axis')
grid on
box on


subplot(3,1,2)
plot(t,sqrt(cov(13:15,:))')
title('Velocity covariance Left Foot')
ylabel('sqrt(cov) [m/s]')
xlabel('time [s]')
legend('x-axis', 'y-axis','z-axis')
grid on
box on

subplot(3,1,3)
plot(t,sqrt(cov(16:18,:))'*180/pi)
title('Heading covariance Left Foot')
ylabel('sqrt(cov) [deg]')
xlabel('time [s]')
legend('Roll', 'Pitch','Yaw')
grid on
box on



%% If the filter also estimates the sensor biases and/or the scalefactor,
%% plot these now

if (strcmp(simdata.scalefactors,'on') && strcmp(simdata.biases,'on'))
    %    Both scale and bias errors included
    figure(7)
    clf
    subplot(2,1,1)
    plot(t,x_h(10:12,:)')
    legend('x-axis','y-axis','z-axis')
    title('Accelerometer bias errors')
    xlabel('time [s]')
    ylabel('Bias [m/s^2]')
    grid on
    box on
    
    subplot(2,1,2)
    plot(t,x_h(13:15,:)'*180/pi)
    legend('x-axis','y-axis','z-axis')
    title('Gyroscope bias errors')
    xlabel('time [s]')
    ylabel('Bias [deg/s]')
    box on
    grid on
    
    figure(8)
    clf
    subplot(2,1,1)
    plot(t,x_h(16:18,:)')
    legend('x-axis','y-axis','z-axis')
    title('Accelerometer scale factor errors')
    xlabel('time [s]')
    box on
    grid on
    
    subplot(2,1,2)
    plot(t,x_h(19:21,:)')
    legend('x-axis','y-axis','z-axis')
    title('Gyroscope scale factor errors')
    xlabel('time [s]')
    box on
    grid on
    
    
elseif strcmp(simdata.scalefactors,'on') && strcmp(simdata.biases,'off')
    %    Scale errors included
    figure(7)
    clf
    subplot(2,1,1)
    plot(t,x_h(10:12,:)')
    legend('x-axis','y-axis','z-axis')
    title('Accelerometer scale factor errors')
    xlabel('time [s]')
    box on
    grid on
    
    subplot(2,1,2)
    plot(t,x_h(13:15,:)'*180/pi)
    legend('x-axis','y-axis','z-axis')
    title('Gyroscope scale factor errors')
    xlabel('time [s]')
    box on
    grid on
    
elseif strcmp(simdata.scalefactors,'off') && strcmp(simdata.biases,'on')
    %    Bias errors included
    figure(7)
    clf
    subplot(2,1,1)
    plot(t,x_h(10:12,:)')
    legend('x-axis','y-axis','z-axis')
    title('Accelerometer bias errors')
    xlabel('time [s]')
    ylabel('Bias [m/s^2]')
    grid on
    box on
    
    subplot(2,1,2)
    plot(t,x_h(13:15,:)'*180/pi)
    legend('x-axis','y-axis','z-axis')
    title('Gyroscope bias errors')
    xlabel('time [s]')
    ylabel('Bias [deg/s]')
    box on
    grid on
end








