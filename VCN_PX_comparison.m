%Clear all previous variables
clear;
clc;

% Declare VICON and Pixhawk data files
VCNfname = 'G:\My Drive\Research\IMAGRS\Debug Quad\Test Flights\20210423\vicon\20210423_flight 2.csv';
PXKfname = 'G:\My Drive\Research\IMAGRS\Debug Quad\Test Flights\20210423\pixhawk\17_51_29.ulg';

% Set axis limits for position plots
x_lim = [-9 9];
y_lim = [-2 2];
z_lim = [0 5];


% Get VICON data formatted

% Prompt user to select .csv file to work with
%disp('Select VICON data file.');
%[FileName,PathName,FilterIndex] = uigetfile('*.csv','Select Vicon Data');
% Add the directory to where this file exists
%addpath(PathName)

% Set framerate based on .csv file and calculate timestep
VCNFrameRate = csvread(VCNfname,1,0,[1,0,1,0]);
timestep = 1/VCNFrameRate;
% Read in .csv file and get size of the matrix
Data = csvread(VCNfname,5,0);
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
[roll, pitch, yaw] = quat2angle(quatObj);
VCNeulObj = [yaw pitch roll];
% Unwrap the euler angles incase we are crossing 180 degrees
VCNeulObj = unwrap(VCNeulObj,[],1);
VCNeulObj(:,4:6) = rad2deg(VCNeulObj(:,1:3));
% Roll in VICON is often flipped, so phase shift if necessary
if VCNeulObj(1,6) > 170
    VCNeulObj(:,6) = VCNeulObj(:,6) - 180;
elseif VCNeulObj(1,6) < -170
    VCNeulObj(:,6) = VCNeulObj(:,6) + 180;
end

% Creat body rates
VCNeulRatesObj = diff(VCNeulObj)/timestep;
VCNvelObj = Data(:,7:9);
VCNvelObj = diff(VCNvelObj)/timestep;
%pad rate matrix, add frame and time columns, and convert position and
%velocity to m and m/s, respectively.
VCNeulObj = [Data(:,1:2) VCNeulObj];
VCNeulRatesObj = [zeros(1,8); Data(2:numFrames,1:2) VCNeulRatesObj];
VCNvelObj = [Data(1,1:2) Data(1,7:9)/1000 0 0 0; Data(2:numFrames,1:2) Data(2:numFrames,7:9)/1000 VCNvelObj/1000];
%calculate body rates
VCNbodyRatesObj = zeros(numFrames,8);
for i=1:numFrames
    VCNbodyRatesObj(i,1) = VCNeulRatesObj(i,1);
    VCNbodyRatesObj(i,2) = VCNeulRatesObj(i,2);
    VCNbodyRatesObj(i,5) = VCNeulRatesObj(i,5) - (sin(VCNeulObj(i,4))*VCNeulRatesObj(i,3));
    VCNbodyRatesObj(i,4) = (cos(VCNeulObj(i,5))*VCNeulRatesObj(i,4)) + (sin(VCNeulObj(i,5))*cos(VCNeulObj(i,4))*VCNeulRatesObj(i,3));
    VCNbodyRatesObj(i,3) = -(sin(VCNeulObj(i,5))*VCNeulRatesObj(i,4)) + (cos(VCNeulObj(i,5))*cos(VCNeulObj(i,4))*VCNeulRatesObj(i,3));
end
% Convert to degrees just for display purposes
VCNbodyRatesObj(:,6:8) = rad2deg(VCNbodyRatesObj(:,3:5));

% clear variables no longer needed
clear VCNfname timestep Data DataN quatObj yaw pitch roll i rows cols numFrames

% Get pixhawk log data formatted
% Prompt user to select .ulg file to work with
%disp('Select the Pixhawk ulg file.');
%[FileName,PathName,FilterIndex] = uigetfile('*.ulg','Select Pixhawk Data');
% Add the directory to where this file exists
%addpath(PathName)

ulogOBJ = ulogreader(PXKfname); %Specify the ULOG file
clear PXKfname

numberOfTopics = size(ulogOBJ.AvailableTopics,1);
msgTable = readTopicMsgs(ulogOBJ);

for i = 1:numberOfTopics
    name = msgTable.TopicNames(i);
    instance = msgTable.InstanceID(i);
    instance = num2str(instance);
    name = name + '_' + instance;
    data = msgTable.TopicMessages(i);
    data = data{1};
    assignin('base',name,data)
    clear name instance data
end
clear i numberOfTopics ulogOBJ msgTable

% Create "seconds" column in pertinent timetables
seconds = double(seconds(vehicle_local_position_0.timestamp));
vehicle_local_position_0 = addvars(vehicle_local_position_0,seconds,'Before','ref_timestamp');
clear seconds
seconds = double(seconds(vehicle_local_position_setpoint_0.timestamp));
vehicle_local_position_setpoint_0 = addvars(vehicle_local_position_setpoint_0,seconds,'Before','x');
clear seconds
seconds = double(seconds(vehicle_attitude_0.timestamp));
vehicle_attitude_0 = addvars(vehicle_attitude_0,seconds,'Before','q');
clear seconds
seconds = double(seconds(vehicle_attitude_setpoint_0.timestamp));
vehicle_attitude_setpoint_0 = addvars(vehicle_attitude_setpoint_0,seconds,'Before','roll_body');
clear seconds
seconds = double(seconds(battery_status_0.timestamp));
battery_status_0 = addvars(battery_status_0,seconds,'Before','voltage_v');
clear seconds

% Create euler angles in the attitude table
eulangles = quat2eul(vehicle_attitude_0.q);
% Rearrange euler angles to get in Yaw Pitch Roll format
eulangles = [-eulangles(:,1)+pi()/2 eulangles(:,2) -eulangles(:,3)];
eulangles = [eulangles rad2deg(eulangles)];
vehicle_attitude_0 = addvars(vehicle_attitude_0,eulangles,'After','q');
clear eulangles

% Create euler angles in degrees in attitude setpoint table
roll_bodyD = rad2deg(vehicle_attitude_setpoint_0.roll_body);
vehicle_attitude_setpoint_0 = addvars(vehicle_attitude_setpoint_0,roll_bodyD,'After','yaw_body');
pitch_bodyD = rad2deg(vehicle_attitude_setpoint_0.pitch_body);
vehicle_attitude_setpoint_0 = addvars(vehicle_attitude_setpoint_0,pitch_bodyD,'After','roll_bodyD');
yaw_bodyD = -rad2deg(vehicle_attitude_setpoint_0.yaw_body)+90;
vehicle_attitude_setpoint_0 = addvars(vehicle_attitude_setpoint_0,yaw_bodyD,'After','pitch_bodyD');

% Offset the VICON and Pixhawk euler angles based on the initial setpoint
% data

offset = VCNeulObj(1,6) - vehicle_attitude_setpoint_0.yaw_bodyD(1);
VCNeulObj(:,6) = VCNeulObj(:,6) - offset;
clear offset
offset = VCNeulObj(1,7) - vehicle_attitude_setpoint_0.pitch_bodyD(1);
VCNeulObj(:,7) = VCNeulObj(:,7) - offset;
clear offset
offset = VCNeulObj(1,8) - vehicle_attitude_setpoint_0.roll_bodyD(1);
VCNeulObj(:,8) = VCNeulObj(:,8) - offset;
clear offset
offset = vehicle_attitude_0.eulangles(1,4) - vehicle_attitude_setpoint_0.yaw_bodyD(1);
vehicle_attitude_0.eulangles(:,4) = vehicle_attitude_0.eulangles(:,4) - offset;
clear offset
offset = vehicle_attitude_0.eulangles(1,5) - vehicle_attitude_setpoint_0.pitch_bodyD(1);
vehicle_attitude_0.eulangles(:,5) = vehicle_attitude_0.eulangles(:,5) - offset;
clear offset
offset = vehicle_attitude_0.eulangles(1,6) - vehicle_attitude_setpoint_0.roll_bodyD(1);
vehicle_attitude_0.eulangles(:,6) = vehicle_attitude_0.eulangles(:,6) - offset;
clear offset

% Creating tiled layout for plots
figure
t1 = tiledlayout(3,3);

% plot raw data
% NOTE: pixhawk coordinate system is rotated relative to VICON (pos-y out
% the nose of the quad, pos-x to the port, and pos-z down). Vicon uses
% pos-x out the nose, pos-y to the port, and pos-z up.
nexttile
plot(VCNvelObj(:,2),VCNvelObj(:,3),'-k','DisplayName','Vicon Data','LineWidth',1.2)
hold on
plot(vehicle_local_position_0.seconds,vehicle_local_position_0.y,'--r','DisplayName','Pixhawk Data','LineWidth',1.2)
ylabel('X-Position (m)');
ylim(x_lim)
title('Raw Data')
set(gca,'FontSize',16)

nexttile(4)
plot(VCNvelObj(:,2),VCNvelObj(:,4),'-k','DisplayName','Vicon Data','LineWidth',1.2)
hold on
plot(vehicle_local_position_0.seconds,vehicle_local_position_0.x,'--r','DisplayName','Pixhawk Data','LineWidth',1.2)
ylabel('Y-Position (m)');
ylim(y_lim)
set(gca,'FontSize',16)
%title('Raw Data')

nexttile(7)
plot(VCNvelObj(:,2),VCNvelObj(:,5),'-k','DisplayName','Vicon Data','LineWidth',1.2)
hold on
plot(vehicle_local_position_0.seconds,-vehicle_local_position_0.z,'--r','DisplayName','Pixhawk Data','LineWidth',1.2)
ylabel('Z-Position (m)');
ylim(z_lim)
xlabel('Time (s)')
set(gca,'FontSize',16)
%title('Raw Data')

% find peaks in x location data
[Vpks,Vlocs,Vwidths,Vproms] = findpeaks(VCNvelObj(:,3),VCNvelObj(:,2));
[Vmax,Vind] = max(Vproms);
[Ppks,Plocs,Pwidths,Pproms] = findpeaks(vehicle_local_position_0.y,vehicle_local_position_0.seconds);
[Pmax,Pind] = max(Pproms);

% Get time and locations of highest peak prominence for each x-location signal
Ptime = Plocs(Pind);
Vtime = Vlocs(Vind);
%PoffsetIndex = find(vehicle_local_position.ref_timestamp == Ptime);
%VoffsetIndex = find(velObj(:,2) == Vtime);
% adjust x, y, and z of pixhawk data to match vicon data
% keep in mind pixhawk y is vicon x and pixhawk x is vicon y, and z is
% opposite sign
% this version of the script assumes the quad is still at the start of the
% recording and uses that position to offset the pixhawk data
offsetx = VCNvelObj(2,3) - vehicle_local_position_0.y(2);
PXx = vehicle_local_position_0.y + offsetx;
% find peaks in y location data
[Vpks,Vlocs,Vwidths,Vproms] = findpeaks(VCNvelObj(:,4),VCNvelObj(:,2));
[Vmax,Vind] = max(Vproms);
[Ppks,Plocs,Pwidths,Pproms] = findpeaks(vehicle_local_position_0.x,vehicle_local_position_0.seconds);
[Pmax,Pind] = max(Pproms);
% Get time and locations of highest peak prominence for each y-location signal
PYtime = Plocs(Pind);
VYtime = Vlocs(Vind);
%PoffsetIndex = find(vehicle_local_position.ref_timestamp == PYtime);
%VoffsetIndex = find(velObj(:,2) == VYtime);
offsety = VCNvelObj(2,4) - vehicle_local_position_0.x(2);
PXy = vehicle_local_position_0.x + offsety;
% find peaks in z location data
[Vpks,Vlocs,Vwidths,Vproms] = findpeaks(VCNvelObj(:,5),VCNvelObj(:,2));
[Vmax,Vind] = max(Vproms);
PXz = -vehicle_local_position_0.z;
[Ppks,Plocs,Pwidths,Pproms] = findpeaks(PXz,vehicle_local_position_0.seconds);
[Pmax,Pind] = max(Pproms);
% Get time and locations of highest peak prominence for each z-location signal
PZtime = Plocs(Pind);
VZtime = Vlocs(Vind);
%PoffsetIndex = find(vehicle_local_position.ref_timestamp == PZtime);
%VoffsetIndex = find(velObj(:,2) == VZtime);
offsetz = VCNvelObj(2,5) - PXz(2);
PXz = PXz + offsetz;

% plot position shifted data
nexttile(2)
plot(VCNvelObj(:,2),VCNvelObj(:,3),'-k','DisplayName','Vicon Data','LineWidth',1.2)
hold on
plot(vehicle_local_position_0.seconds,PXx,'--r','DisplayName','Pixhawk Data','LineWidth',1.2)
ylim(x_lim)
set(gca,'FontSize',16)
title('Position Shifted')

nexttile(5)
plot(VCNvelObj(:,2),VCNvelObj(:,4),'-k','DisplayName','Vicon Data','LineWidth',1.2)
hold on
plot(vehicle_local_position_0.seconds,PXy,'--r','DisplayName','Pixhawk Data','LineWidth',1.2)
ylim(y_lim)
set(gca,'FontSize',16)
%title('Position Shifted')

nexttile(8)
plot(VCNvelObj(:,2),VCNvelObj(:,5),'-k','DisplayName','Vicon Data','LineWidth',1.2)
hold on
plot(vehicle_local_position_0.seconds,PXz,'--r','DisplayName','Pixhawk Data','LineWidth',1.2)
ylim(z_lim)
xlabel('Time (s)')
set(gca,'FontSize',16)
%title('Position Shifted')

% get difference in time and shift data
shift = Ptime - Vtime;
vehicle_local_position_0.seconds = vehicle_local_position_0.seconds - shift;
vehicle_local_position_setpoint_0.seconds = vehicle_local_position_setpoint_0.seconds - shift;
vehicle_attitude_0.seconds = vehicle_attitude_0.seconds - shift;
vehicle_attitude_setpoint_0.seconds = vehicle_attitude_setpoint_0.seconds - shift;
battery_status_0.seconds = battery_status_0.seconds - shift;

% Plot position and time shifted data
nexttile(3)
plot(VCNvelObj(:,2),VCNvelObj(:,3),'-k','DisplayName','Vicon Data','LineWidth',1.2)
hold on
plot(vehicle_local_position_0.seconds,PXx,'--r','DisplayName','Pixhawk Data','LineWidth',1.2)
plot(vehicle_local_position_setpoint_0.seconds,vehicle_local_position_setpoint_0.y+offsetx,'-g','DisplayName','Setpoint','LineWidth',1.2)
ylim(x_lim)
set(gca,'FontSize',16)
title('Time Shifted')

nexttile(6)
plot(VCNvelObj(:,2),VCNvelObj(:,4),'-k','DisplayName','Vicon Data','LineWidth',1.2)
hold on
plot(vehicle_local_position_0.seconds,PXy,'--r','DisplayName','Pixhawk Data','LineWidth',1.2)
plot(vehicle_local_position_setpoint_0.seconds,vehicle_local_position_setpoint_0.x+offsety,'-g','DisplayName','Setpoint','LineWidth',1.2)
ylim(y_lim)
set(gca,'FontSize',16)
%title('Time Shifted')

nexttile(9)
plot(VCNvelObj(:,2),VCNvelObj(:,5),'-k','DisplayName','Vicon Data','LineWidth',1.2)
hold on
plot(vehicle_local_position_0.seconds,PXz,'--r','DisplayName','Pixhawk Data','LineWidth',1.2)
plot(vehicle_local_position_setpoint_0.seconds,-vehicle_local_position_setpoint_0.z+offsetz,'-g','DisplayName','Setpoint','LineWidth',1.2)
ylim(z_lim)
xlabel('Time (s)')
set(gca,'FontSize',16)
%title('Time Shifted')

lg = legend;
lg.Layout.Tile = 'South';
lg.Orientation = 'Horizontal';
clear lg

figure
t2 = tiledlayout(1,3);

nexttile(1)
plot(VCNeulObj(:,2),VCNeulObj(:,6),'-k','DisplayName','Vicon Data','LineWidth',1.2)
hold on
plot(vehicle_attitude_0.seconds,vehicle_attitude_0.eulangles(:,4),'--r','DisplayName','Pixhawk Data','LineWidth',1.2)
plot(vehicle_attitude_setpoint_0.seconds,vehicle_attitude_setpoint_0.yaw_bodyD,'-g','DisplayName','Setpoint','LineWidth',1.2)
xlabel('Time (s)');
ylabel('Yaw Angle (\circ)');
set(gca,'FontSize',16)

nexttile(2)
plot(VCNeulObj(:,2),VCNeulObj(:,7),'-k','DisplayName','Vicon Data','LineWidth',1.2)
hold on
plot(vehicle_attitude_0.seconds,vehicle_attitude_0.eulangles(:,5),'--r','DisplayName','Pixhawk Data','LineWidth',1.2)
plot(vehicle_attitude_setpoint_0.seconds,vehicle_attitude_setpoint_0.pitch_bodyD,'-g','DisplayName','Setpoint','LineWidth',1.2)
xlabel('Time (s)');
ylabel('Pitch Angle (\circ)');
set(gca,'FontSize',16)

nexttile(3)
plot(VCNeulObj(:,2),VCNeulObj(:,8),'-k','DisplayName','Vicon Data','LineWidth',1.2)
hold on
plot(vehicle_attitude_0.seconds,vehicle_attitude_0.eulangles(:,6),'--r','DisplayName','Pixhawk Data','LineWidth',1.2)
plot(vehicle_attitude_setpoint_0.seconds,vehicle_attitude_setpoint_0.roll_bodyD,'-g','DisplayName','Setpoint','LineWidth',1.2)
xlabel('Time (s)');
ylabel('Roll Angle (\circ)');
set(gca,'FontSize',16)

lg = legend;
lg.Layout.Tile = 'South';
lg.Orientation = 'Horizontal';

figure
plot(battery_status_0.seconds,battery_status_0.voltage_v,'-k','DisplayName','Raw Voltage','LineWidth',1.2)
hold on
plot(battery_status_0.seconds,battery_status_0.voltage_filtered_v,'-r','DisplayName','Filtered Voltage','LineWidth',1.2)
plot(battery_status_0.seconds,battery_status_0.discharged_mah./1000,'-b','DisplayName','Discharged (Ah)','LineWidth',1.2)
legend
set(gca,'FontSize',16)

