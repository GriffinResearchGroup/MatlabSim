% This script takes quaternion data from a .csv file that is exported from 
% the Vicon Tracker software and creates matrices of euler angles, euler 
% rates, body rates, and velocity. It also creates a tiled animation
% playback of the flight. There is included the option to save the
% animation as a file. The playback is resource intensive so is not capable
% of being real time, but the saved file does produce real time footage.
%
% The format of the each matrix is as follows:
% 
% eulObj
% FRAME  TIME  YAW(rad) PITCH(rad) ROLL(rad) YAW(deg)  PITCH(deg)  ROLL(deg)
% 
% eulRatesObj
% FRAME  TIME  YAWRATE(rad/s)  PITCHRATE(rad/s)  ROLLRATE(rad/s)  YAWRATE(deg/s)  PITCHRATE(deg/s)  ROLLRATE(deg/s)
% 
% bodyRatesObj
% FRAME  TIME  YAWRATE(rad/s)  PITCHRATE(rad/s)  ROLLRATE(rad/s)  YAWRATE(deg/s)  PITCHRATE(deg/s)  ROLLRATE(deg/s)
%
% velObj
% FRAME  TIME  X(m)  Y(m)  Z(m)  X-VEL(m/s)  Y-VEL(m/s)  Z-VEL(m/s)

%Clear all previous variables
clear;
clc;
% Prompt user to select .csv file to work with
[FileName,PathName,FilterIndex] = uigetfile('*.csv');
% Add the directory to where this file exists
addpath(PathName)
% Set framerate based on .csv file and calculate timestep
FrameRate = csvread(FileName,1,0,[1,0,1,0]);
timestep = 1/FrameRate;
% Read in .csv file and get size of the matrix
Data = csvread(FileName,5,0);
[numFrames, cols] = size(Data);
% csvread will apply zeros to all blank cells, replace these zeros with NaN
% values to aid in plotting the data
DataN = Data(:,2:cols);
DataN(DataN==0) = nan;
Data = [Data(:,1) DataN];
% Loop through matrix to add time column
time(1) = 0;
for i = 2:numFrames
    time(i) = time(i-1) + timestep;
end
time = time';
Data = [Data(:,1) time Data(:,3:cols)];

%Convert from quaternions to euler angles for the object and calculate euler rates and velocities
quatObj = Data(:,3:6);
[yaw, pitch, roll] = quat2angle(quatObj);
eulObj = [yaw pitch roll];
eulObj(:,4:6) = rad2deg(eulObj(:,1:3));
eulRatesObj = diff(eulObj)/timestep;
velObj = Data(:,7:9);
velObj = diff(velObj)/timestep;
%pad rate matrix, add frame and time columns, and convert position and
%velocity to m and m/s, respectively.
eulObj = [Data(:,1:2) eulObj];
eulRatesObj = [zeros(1,8); Data(2:numFrames,1:2) eulRatesObj];
velObj = [Data(1,1:2) Data(1,7:9)/1000 0 0 0; Data(2:numFrames,1:2) Data(2:numFrames,7:9)/1000 velObj/1000];
%calculate body rates
for i=1:numFrames
    bodyRatesObj(i,1) = eulRatesObj(i,1);
    bodyRatesObj(i,2) = eulRatesObj(i,2);
    bodyRatesObj(i,5) = eulRatesObj(i,5) - (sin(eulObj(i,4))*eulRatesObj(i,3));
    bodyRatesObj(i,4) = (cos(eulObj(i,5))*eulRatesObj(i,4)) + (sin(eulObj(i,5))*cos(eulObj(i,4))*eulRatesObj(i,3));
    bodyRatesObj(i,3) = -(sin(eulObj(i,5))*eulRatesObj(i,4)) + (cos(eulObj(i,5))*cos(eulObj(i,4))*eulRatesObj(i,3));
end
% Convert to degrees just for display purposes
bodyRatesObj(:,6:8) = rad2deg(bodyRatesObj(:,3:5));

%Create time series data for Aero object
tdata = [velObj(:,2) -velObj(:,3) -velObj(:,5) -velObj(:,4) eulObj(:,3) eulObj(:,5) eulObj(:,4)];
% Remove rows that contain NaN
tdata(any(isnan(tdata), 2), :) = [];

%Create aero body from ac3d file
h = Aero.Body;
h.load('quad.ac','Ac3d');

%Plot trajectory in tunnel and Euler angles and show simulation of flight
figure
tl1 = tiledlayout(2,2);

%First tile is trajectory in tunnel
ax1 = nexttile(tl1);
scatter3(velObj(:,3),velObj(:,4),velObj(:,5),10,velObj(:,2))
grid on
axis equal
xlim([0 30])
ylim([-2.5 2.5])
zlim([-0.5 5])
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
set(ax1,'FontSize',16);
c = colorbar;
c.Label.String = 'Time at position in seconds';
hold on

%Second tile is Euler angles
ax2 = nexttile(tl1,3);
%Correct yaw angle if needed
yawTemp = unwrap(eulObj(:,3));
yawTemp = rad2deg(yawTemp + pi);
%scatter(eulObj(:,2),yawTemp,'filled','o');
hold on
%scatter(eulObj(:,2),eulObj(:,7),'filled','s');
%scatter(eulObj(:,2),eulObj(:,8),'filled','^');
plot(eulObj(:,2),yawTemp,'k-',eulObj(:,2),eulObj(:,7),'r--',eulObj(:,2),eulObj(:,8),'b:')
ylim([-45 45])
xlabel('Time (s)')
ylabel('Body Angle (deg)')
legend('Roll', 'Pitch', 'Yaw')
set(ax2,'FontSize',16);


%Set tiled layour to be 3 x 1
tl2 = tiledlayout(tl1,3,1);
tl2.Layout.Tile = 2;
tl2.Layout.TileSpan = [2 1];

%First tile will show side view
ax3 = nexttile(tl2);
h.generatePatches(ax3);
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
camup([0 0 1])
axis equal
%set(gca,'XDir','reverse')
grid on
%hold on
xlim([0 30])
ylim([-2.5 2.5])
zlim([-0.5 5])
view([0 0])
set(ax3,'FontSize',16);
h.TimeSeriesSource = tdata;
h.update(0)
%campos([60 -80 78]);

%Create second instance of aero body for second tile
k = Aero.Body;
k.load('quad.ac','Ac3d');

%Create second tile that will show top view
ax4 = nexttile(tl2);
k.generatePatches(ax4);
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
%camup([0 0 1])
axis equal
%set(gca,'XDir','reverse')
grid on
%hold on
xlim([0 30])
ylim([-2.5 2.5])
zlim([-0.5 5])
set(ax4,'FontSize',16);
%view([270 0])
k.TimeSeriesSource = tdata;
k.update(0)

%Create third instance of aero body for third tile
l = Aero.Body;
l.load('quad.ac','Ac3d');

%Create third tile that will show down range view
ax5 = nexttile(tl2);
l.generatePatches(ax5);
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
camup([0 0 1])
axis equal
%set(gca,'XDir','reverse')
grid on
%hold on
xlim([0 30])
ylim([-2.5 2.5])
zlim([-0.5 5])
view([270 0])
set(ax5,'FontSize',16);
l.TimeSeriesSource = tdata;
l.update(0)

[numRows,numCols] = size(tdata);
r = rateControl(FrameRate);
%Let user decide to save animation
prompt = 'Do you want to save the animation? Y/N [N]: ';
strSave = input(prompt,'s');
if strSave == 'Y'
    % Initialize video
    myVideo = VideoWriter('animationFile');
    myVideo.FrameRate = FrameRate;
    open(myVideo)
end
%Pause to let user arrange windows before animation starts
disp('Press any key to start animation.');
pause
for i = 1:numRows
    ts = tdata(i,1);
    h.update(ts)
    k.update(ts)
    l.update(ts)
    s1 = xline(ax2,eulObj(i,2),'HandleVisibility','off');
    s2 = scatter3(ax1,velObj(i,3),velObj(i,4),velObj(i,5),50,'r','filled');
    if strSave == 'Y'
        %Save animated plot to video file
        frame = getframe(gcf); %get frame
        writeVideo(myVideo, frame);
    end
    waitfor(r);
    delete(s1)
    delete(s2)
end

if strSave == 'Y'
    close(myVideo)
end