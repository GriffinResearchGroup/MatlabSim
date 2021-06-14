% Script to analyze power consumption of quad rotor during hover test flights.

%Clear all previous variables
clear;
clc;

% Declare Pixhawk data files
PXKfname(1,1) = "Replicate 1";
PXKfname(1,2) = "G:\My Drive\Research\IMAGRS\Debug Quad\Test Flights\hover_test\rep1\pixhawk\18_31_33.ulg";
PXKfname(2,1) = "Replicate 2";
PXKfname(2,2) = "G:\My Drive\Research\IMAGRS\Debug Quad\Test Flights\hover_test\rep2\pixhawk\18_39_03.ulg";
PXKfname(3,1) = "Replicate 3";
PXKfname(3,2) = "G:\My Drive\Research\IMAGRS\Debug Quad\Test Flights\hover_test\rep3\pixhawk\16_51_30.ulg";
PXKfname(4,1) = "Y Test";
PXKfname(4,2) = "G:\My Drive\Research\IMAGRS\Debug Quad\Test Flights\hover_test\ytest\pixhawk\19_00_29.ulg";

% Declare line styles for 2:m test flights
LS = ["-k" "--b" "-.r" "-m"];
% Declare start shift for linear curve fit
ndg = 100;

% Initialize some matrices
[m,~] = size(PXKfname);
x_dist = zeros(1,m);
y_dist = zeros(1,m);
z_dist = zeros(1,m);
SD = zeros(m,3);

% loop through log files and create timetables for each dataset
% the topics are indexed in alphabetical order, ie 1 = battery_status, 2 =
% vehicle_attitude, 3 = vehicle_land_detected, and 4 =
% vehicle_local_position
for i = 1:m
    ulogOBJ = ulogreader(PXKfname(i,2)); %Specify the ULOG file
    msgTable = readTopicMsgs(ulogOBJ,'TopicName',{'vehicle_local_position','battery_status','vehicle_land_detected','vehicle_attitude'});
    [n,~] = size(msgTable);
    for j = 1:n
        topic = msgTable.TopicNames(j);
        data = msgTable.TopicMessages(j);
        data = data{1};
        seconds = double(seconds(data.timestamp));
        if j == 1
            start = seconds(1);
        end
        seconds = seconds - start;
        data = addvars(data,seconds,'Before',1);
        % Normalize discharge so it starts at 0
        if topic == "battery_status"
            data.discharged_mah = data.discharged_mah - data.discharged_mah(1);
            % Calculate Watt-Hours and add to dataset
            discharged_Wh = data.voltage_v .* (data.discharged_mah./1000);
            data = addvars(data,discharged_Wh,'After','discharged_mah');
        elseif topic == "vehicle_local_position"
            % Calculate total distance traveled in each direction
            deltaX = diff(data.y);
            deltaY = diff(data.x);
            deltaZ = diff(data.z);
            x_dist(i) = sumabs(deltaX);
            y_dist(i) = sumabs(deltaY);
            z_dist(i) = sumabs(deltaZ);
            % Calculate standard deviation for each axis for each replicate
            SD(i,1) = std(data.y(500:end-500));
            SD(i,2) = std(data.x(500:end-500));
            SD(i,3) = std(data.z(500:end-500));
        end
        %assignin('base',topic,data)
        Dataset{i,j} = data;
        clear seconds data deltaX deltaY deltaZ topic
    end
    clear ulogOBJ msgTable
end
clear i j n

% Create discharge_mAh vs. time plot
figure
plot(Dataset{1,1}.seconds,Dataset{1,1}.discharged_mah,'-k','LineWidth',1.0,'DisplayName',PXKfname(1,1))
grid on
xlabel('Time (sec)')
ylabel('Energy Discharged (mAh)')
ax = gca;
ax.FontSize = 20;
legend('Location','southeast')
fig = gcf;
fig.Position = [40 40 1000 800];
hold on
% Conduct linear fit to find discharge per unit time
pmAh_time = zeros(m,2);
pmAh_time(1,:) = polyfit(Dataset{1,1}.seconds(ndg:end-ndg),Dataset{1,1}.discharged_mah(ndg:end-ndg),1);
x = linspace(Dataset{1,1}.seconds(ndg),Dataset{1,1}.seconds(end-ndg));
f = polyval(pmAh_time(1,:),x);
plot(x,f,':','Color','#666666','LineWidth',2.0,'HandleVisibility','off')
clear x f
for i = 2:m
    plot(Dataset{i,1}.seconds,Dataset{i,1}.discharged_mah,LS(i),'LineWidth',1.0,'DisplayName',PXKfname(i,1))
    pmAh_time(i,:) = polyfit(Dataset{i,1}.seconds(ndg:end-ndg),Dataset{i,1}.discharged_mah(ndg:end-ndg),1);
    x = linspace(Dataset{i,1}.seconds(ndg),Dataset{i,1}.seconds(end-ndg));
    f = polyval(pmAh_time(i,:),x);
    if i == m
        plot(x,f,':','Color','#666666','LineWidth',2.0,'DisplayName','Linear Fit')
    else
        plot(x,f,':','Color','#666666','LineWidth',2.0,'HandleVisibility','off')
    end
    clear x f
end

% Create discharge_Wh vs. time plot
figure
plot(Dataset{1,1}.seconds,Dataset{1,1}.discharged_Wh,'-k','LineWidth',1.0,'DisplayName',PXKfname(1,1))
grid on
xlabel('Time (sec)')
ylabel('Energy Discharged (Wh)')
ax = gca;
ax.FontSize = 20;
legend('Location','southeast')
fig = gcf;
fig.Position = [40 40 1000 800];
hold on
% Conduct linear fit to find discharge per unit time
pWh_time = zeros(m,2);
pWh_time(1,:) = polyfit(Dataset{1,1}.seconds(ndg:end-ndg),Dataset{1,1}.discharged_Wh(ndg:end-ndg),1);
x = linspace(Dataset{1,1}.seconds(ndg),Dataset{1,1}.seconds(end-ndg));
f = polyval(pWh_time(1,:),x);
plot(x,f,':','Color','#666666','LineWidth',2.0,'HandleVisibility','off')
clear x f
for i = 2:m
    plot(Dataset{i,1}.seconds,Dataset{i,1}.discharged_Wh,LS(i),'LineWidth',1.0,'DisplayName',PXKfname(i,1))
    pWh_time(i,:) = polyfit(Dataset{i,1}.seconds(ndg:end-ndg),Dataset{i,1}.discharged_Wh(ndg:end-ndg),1);
    x = linspace(Dataset{i,1}.seconds(ndg),Dataset{i,1}.seconds(end-ndg));
    f = polyval(pWh_time(i,:),x);
    if i == m
        plot(x,f,':','Color','#666666','LineWidth',2.0,'DisplayName','Linear Fit')
    else
        plot(x,f,':','Color','#666666','LineWidth',2.0,'HandleVisibility','off')
    end
    clear x f
end

% Plot altitude just to make sure files were entered correctly
figure
plot(Dataset{1,4}.seconds,-Dataset{1,4}.z,'-k','LineWidth',1.0,'DisplayName',PXKfname(1,1))
grid on
xlabel('Time (sec)')
ylabel('Altitude (m)')
ax = gca;
ax.FontSize = 20;
legend('Location','northeast')
fig = gcf;
fig.Position = [40 40 1000 800];
hold on
for i = 2:m
    plot(Dataset{i,4}.seconds,-Dataset{i,4}.z,LS(i),'LineWidth',1.0,'DisplayName',PXKfname(i,1))
end
SD
pmAh_time
pWh_time