%Clear all previous variables
clear;
clc;

% Get VICON data formatted

% Prompt user to select .csv file to work with
disp('Select VICON data file.');
[FileName,PathName,FilterIndex] = uigetfile('*.csv','Select Vicon Data');
% Add the directory to where this file exists
addpath(PathName)
% Set framerate based on .csv file and calculate timestep
VCNFrameRate = csvread(FileName,1,0,[1,0,1,0]);
timestep = 1/VCNFrameRate;
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

% Get pixhawk log data formatted
% Prompt user to select .ulg file to work with
disp('Select the Pixhawk ulg file.');
[FileName,PathName,FilterIndex] = uigetfile('*.ulg','Select Pixhawk Data');
% Add the directory to where this file exists
addpath(PathName)

ulogOBJ = ulogreader(FileName); %Specify the ULOG file

numberOfTopics = size(ulogOBJ.AvailableTopics,1);
msgTable = readTopicMsgs(ulogOBJ);

for i = 1:numberOfTopics
name = msgTable.TopicNames(i);
data = msgTable.TopicMessages(i);
data = data{1};
assignin('base',name,data)
clear name data
end
clear i numberOfTopics ulogOBJ msgTable

% Use empty ref_timestamp column to create array of seconds
vehicle_local_position.ref_timestamp = double(seconds(vehicle_local_position.timestamp - vehicle_local_position.timestamp(1)));
% Get data rate for pixhawk
PXfr = mean(1./diff(vehicle_local_position.ref_timestamp));

% Resample to highest frequency and put pixhawk coordinate system in VICON
% coordinate system
if VCNFrameRate > PXfr
    [P,Q] = rat(VCNFrameRate/PXfr);
    PXx = resample(double(vehicle_local_position.y),P,Q);
    PXy = resample(double(vehicle_local_position.x),P,Q);
    PXz = -resample(double(vehicle_local_position.z),P,Q);
    VNx = velObj(:,3);
    VNy = velObj(:,4);
    VNz = velObj(:,5);
    Fr = VCNFrameRate;
else
    [P,Q] = rat(PXfr/VCNFrameRate);
    PXx = vehicle_local_position.y;
    PXy = vehicle_local_position.x;
    PXz = -vehicle_local_position.z;
    VNx = resample(velObj(:,3),P,Q);
    VNy = resample(velObj(:,4),P,Q);
    VNz = resample(velObj(:,5),P,Q);
    Fr = PXfr;
end

% Align the signals
VNx = alignsignals(VNx,PXx);
VNy = alignsignals(VNy,PXy);
VNz = alignsignals(VNz,PXz);

figure
tl1 = tiledlayout(3,1);
ax1 = nexttile;
plot((0:numel(VNx)-1)/Fr,VNx);
grid on
hold on
plot((0:numel(PXx)-1)/Fr,PXx);
legend('Vicon','Pixhawk');
xlabel('time (s)');
ylabel('x (m)');
ax2 = nexttile;
plot((0:numel(VNy)-1)/Fr,VNy);
grid on
hold on
plot((0:numel(PXy)-1)/Fr,PXy);
legend('Vicon','Pixhawk');
xlabel('time (s)');
ylabel('y (m)');
ax3 = nexttile;
plot((0:numel(VNz)-1)/Fr,VNz);
grid on
hold on
plot((0:numel(PXz)-1)/Fr,PXz);
legend('Vicon','Pixhawk');
xlabel('time (s)');
ylabel('y (m)');