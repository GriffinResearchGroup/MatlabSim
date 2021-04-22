%Clear all previous variables
clear;
clc;

% Get VICON data formatted

% Prompt user to select .csv file to work with
%disp('Select VICON data file.');
%[FileName,PathName,FilterIndex] = uigetfile('*.csv','Select Vicon Data');
% Add the directory to where this file exists
%addpath(PathName)
% Set framerate based on .csv file and calculate timestep
FileName = 'G:\My Drive\Research\IMAGRS\Debug Quad\Test Flights\20210409\Vicon\20210409test 6.csv';
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
time = zeros(1,numFrames);
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
bodyRatesObj = zeros(numFrames,8);
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
%disp('Select the Pixhawk ulg file.');
%[FileName,PathName,FilterIndex] = uigetfile('*.ulg','Select Pixhawk Data');
% Add the directory to where this file exists
%addpath(PathName)

FileName = 'G:\My Drive\Research\IMAGRS\Debug Quad\Test Flights\20210409\Pixhawk\20_29_45.ulg';
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

% Creating tiled layout for plots
figure
tiledlayout(3,3)

% plot raw data
nexttile
plot(velObj(:,2),velObj(:,3),'DisplayName','Vicon Data')
hold on
plot(vehicle_local_position.ref_timestamp,vehicle_local_position.y,'DisplayName','Pixhawk Data')
xlabel('Time (s)');
ylabel('X-Position (m)');
legend
title('Raw Data')

nexttile(4)
plot(velObj(:,2),velObj(:,4),'DisplayName','Vicon Data')
hold on
plot(vehicle_local_position.ref_timestamp,vehicle_local_position.x,'DisplayName','Pixhawk Data')
xlabel('Time (s)');
ylabel('Y-Position (m)');
legend
%title('Raw Data')

nexttile(7)
plot(velObj(:,2),velObj(:,5),'DisplayName','Vicon Data')
hold on
plot(vehicle_local_position.ref_timestamp,-vehicle_local_position.z,'DisplayName','Pixhawk Data')
xlabel('Time (s)');
ylabel('Z-Position (m)');
legend
%title('Raw Data')

% find peaks in x location data
[Vpks,Vlocs,Vwidths,Vproms] = findpeaks(velObj(:,3),velObj(:,2));
[Vmax,Vind] = max(Vproms);
[Ppks,Plocs,Pwidths,Pproms] = findpeaks(vehicle_local_position.y,vehicle_local_position.ref_timestamp);
[Pmax,Pind] = max(Pproms);

% Get time and locations of highest peak prominence for each x-location signal
Ptime = Plocs(Pind);
Vtime = Vlocs(Vind);
PoffsetIndex = find(vehicle_local_position.ref_timestamp == Ptime);
VoffsetIndex = find(velObj(:,2) == Vtime);
% adjust x, y, and z of pixhawk data to match vicon data
% keep in mind pixhawk y is vicon x and pixhawk x is vicon y
offsetx = velObj(VoffsetIndex,3) - vehicle_local_position.y(PoffsetIndex);
PXx = vehicle_local_position.y + offsetx;
% find peaks in y location data
[Vpks,Vlocs,Vwidths,Vproms] = findpeaks(velObj(:,4),velObj(:,2));
[Vmax,Vind] = max(Vproms);
[Ppks,Plocs,Pwidths,Pproms] = findpeaks(vehicle_local_position.x,vehicle_local_position.ref_timestamp);
[Pmax,Pind] = max(Pproms);
% Get time and locations of highest peak prominence for each y-location signal
PYtime = Plocs(Pind);
VYtime = Vlocs(Vind);
PoffsetIndex = find(vehicle_local_position.ref_timestamp == PYtime);
VoffsetIndex = find(velObj(:,2) == VYtime);
offsety = velObj(VoffsetIndex,4) - vehicle_local_position.x(PoffsetIndex);
PXy = vehicle_local_position.x + offsety;
% find peaks in z location data
[Vpks,Vlocs,Vwidths,Vproms] = findpeaks(velObj(:,5),velObj(:,2));
[Vmax,Vind] = max(Vproms);
PXz = -vehicle_local_position.z;
[Ppks,Plocs,Pwidths,Pproms] = findpeaks(PXz,vehicle_local_position.ref_timestamp);
[Pmax,Pind] = max(Pproms);
% Get time and locations of highest peak prominence for each z-location signal
PZtime = Plocs(Pind);
VZtime = Vlocs(Vind);
PoffsetIndex = find(vehicle_local_position.ref_timestamp == PZtime);
VoffsetIndex = find(velObj(:,2) == VZtime);
offsetz = velObj(VoffsetIndex,5) - PXz(PoffsetIndex);
PXz = PXz + offsetz;

% plot position shifted data
nexttile(2)
plot(velObj(:,2),velObj(:,3),'DisplayName','Vicon Data')
hold on
plot(vehicle_local_position.ref_timestamp,PXx,'DisplayName','Pixhawk Data')
xlabel('Time (s)');
ylabel('X-Position (m)');
legend
title('Position Shifted')

nexttile(5)
plot(velObj(:,2),velObj(:,4),'DisplayName','Vicon Data')
hold on
plot(vehicle_local_position.ref_timestamp,PXy,'DisplayName','Pixhawk Data')
xlabel('Time (s)');
ylabel('Y-Position (m)');
legend
%title('Position Shifted')

nexttile(8)
plot(velObj(:,2),velObj(:,5),'DisplayName','Vicon Data')
hold on
plot(vehicle_local_position.ref_timestamp,PXz,'DisplayName','Pixhawk Data')
xlabel('Time (s)');
ylabel('Z-Position (m)');
legend
%title('Position Shifted')

% get difference in time and shift data
shift = Ptime - Vtime;
vehicle_local_position.ref_timestamp = vehicle_local_position.ref_timestamp - shift;

% Plot position and time shifted data
nexttile(3)
plot(velObj(:,2),velObj(:,3),'DisplayName','Vicon Data')
hold on
plot(vehicle_local_position.ref_timestamp,PXx,'DisplayName','Pixhawk Data')
xlabel('Time (s)');
ylabel('X-Position (m)');
legend
title('Time Shifted')

nexttile(6)
plot(velObj(:,2),velObj(:,4),'DisplayName','Vicon Data')
hold on
plot(vehicle_local_position.ref_timestamp,PXy,'DisplayName','Pixhawk Data')
xlabel('Time (s)');
ylabel('Y-Position (m)');
legend
%title('Time Shifted')

nexttile(9)
plot(velObj(:,2),velObj(:,5),'DisplayName','Vicon Data')
hold on
plot(vehicle_local_position.ref_timestamp,PXz,'DisplayName','Pixhawk Data')
xlabel('Time (s)');
ylabel('Z-Position (m)');
legend
%title('Time Shifted')