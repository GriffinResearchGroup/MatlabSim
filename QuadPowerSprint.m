% Script to analyze power consumption of quad rotor during sprint test flights.

%Clear all previous variables
clear;
clc;

% Declare Pixhawk data files
PXKfname(1,1) = "Replicate 1";
PXKfname(1,2) = "G:\My Drive\Research\IMAGRS\Debug Quad\Test Flights\sprint_test\rep1\pixhawk\14_19_14.ulg";
PXKfname(2,1) = "Replicate 2";
PXKfname(2,2) = "G:\My Drive\Research\IMAGRS\Debug Quad\Test Flights\sprint_test\rep2\pixhawk\18_01_32.ulg";
%PXKfname(3,1) = "5.0 m/sec";
%PXKfname(3,2) = "G:\My Drive\Research\IMAGRS\Debug Quad\Test Flights\20210611\pixhawk\18_44_36.ulg";

% Declare line styles for 2:m test flights
%LS = ["-k" "--b" "-.r"];
LS = ["-k" "--b"];
% Declare start shift for linear curve fit
ndg = 100;

% Initialize some matrices
[m,~] = size(PXKfname);
x_dist = zeros(1,m);
y_dist = zeros(1,m);
z_dist = zeros(1,m);
u = zeros(m,2);
v = zeros(m,2);
w = zeros(m,2);

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
            temp = 0.5 * (data.current_a(1:end-1) + data.current_a(2:end));
            temp = [0; temp];
            [nn,~] = size(data);
            disch = zeros(nn,1);
            for k = 1:nn
                if k == 1
                    disch(k) = 0;
                else
                    disch(k) = (temp(k)*(seconds(k)-seconds(k-1))*(1000/3600))+disch(k-1);
                end
            end
            data = addvars(data,disch,'Before','discharged_mah','NewVariableNames',{'discharged_cust'});
            clear temp k nn
        elseif topic == "vehicle_local_position"
            % Calculate total distance traveled in each direction
            deltaX = diff(data.y);
            deltaY = diff(data.x);
            deltaZ = diff(data.z);
            x_dist(i) = sumabs(deltaX);
            y_dist(i) = sumabs(deltaY);
            z_dist(i) = sumabs(deltaZ);
            
            % Create array that is just cumulating distance in each 
            % coordinate direction
            [o,~] = size(data);
            cumX = zeros(o,1);
            cumY = zeros(o,1);
            cumZ = zeros(o,1);
            for k = 2:o
                cumX(k) = abs(data.y(k) - data.y(k-1)) + cumX(k-1);
                cumY(k) = abs(data.x(k) - data.x(k-1)) + cumY(k-1);
                cumZ(k) = abs(data.z(k) - data.z(k-1)) + cumZ(k-1);
            end
            clear o k
            data = addvars(data,cumX,cumY,cumZ,'Before','x');
            
            % Subsample vehicle_local_position to get distance data
            % correlated to power data. Since the topics are indexed in 
            % alphabetical order I know the battery_status topic has 
            % already been added to "Dataset"
            [o,~] = size(Dataset{i,1});
            powX = zeros(o,4);
            for p_time = 1:o
                powX(p_time,1) = Dataset{i,1}.seconds(p_time);
                [minValue,index] = min(abs(powX(p_time,1) - data.seconds));
                powX(p_time,2) = data.cumX(index);
                powX(p_time,3) = data.cumY(index);
                powX(p_time,4) = data.cumZ(index);
            end
            clear o p cumX
            Dataset{i,1} = addvars(Dataset{i,1},powX(:,2),powX(:,3),powX(:,4),'After','seconds','NewVariableNames',{'cumX','cumY','cumZ'});
            % Calculate average and maximum velocities
            u_avg = meanabs(data.vy);
            v_avg = meanabs(data.vx);
            w_avg = meanabs(data.vz);
            u_max = max(abs(data.vy));
            v_max = max(abs(data.vx));
            w_max = max(abs(data.vz));
            u(i,:) = [u_avg u_max];
            v(i,:) = [v_avg v_max];
            w(i,:) = [w_avg w_max];
            clear u_avg v_avg w_avg u_max v_max w_max
        end
        %assignin('base',topic,data)
        Dataset{i,j} = data;
        clear seconds data cumX deltaX deltaY deltaZ topic
    end
    clear ulogOBJ msgTable
end
clear i j n

% Create discharge_mah vs. time plot
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

% Create discharge_mAh vs. distance plot
figure
plot(Dataset{1,1}.cumX,Dataset{1,1}.discharged_mah,'-k','LineWidth',1.0,'DisplayName',PXKfname(1,1))
grid on
xlabel('Distance (m)')
ylabel('Energy Discharged (mAh)')
ax = gca;
ax.FontSize = 20;
legend('Location','southeast')
fig = gcf;
fig.Position = [40 40 1000 800];
hold on
% Conduct linear fit to find discharge per unit length
pmAh_dist = zeros(m,2);
pmAh_dist(1,:) = polyfit(Dataset{1,1}.cumX(ndg:end-ndg),Dataset{1,1}.discharged_mah(ndg:end-ndg),1);
x = linspace(Dataset{1,1}.cumX(ndg),Dataset{1,1}.cumX(end-ndg));
f = polyval(pmAh_dist(1,:),x);
plot(x,f,':','Color','#666666','LineWidth',2.0,'HandleVisibility','off')
clear x f
for i = 2:m
    plot(Dataset{i,1}.cumX,Dataset{i,1}.discharged_mah,LS(i),'LineWidth',1.0,'DisplayName',PXKfname(i,1))
    pmAh_dist(i,:) = polyfit(Dataset{i,1}.cumX(ndg:end-ndg),Dataset{i,1}.discharged_mah(ndg:end-ndg),1);
    x = linspace(Dataset{i,1}.cumX(ndg),Dataset{i,1}.cumX(end-ndg));
    f = polyval(pmAh_dist(i,:),x);
    if i == m
        plot(x,f,':','Color','#666666','LineWidth',2.0,'DisplayName','Linear Fit')
    else
        plot(x,f,':','Color','#666666','LineWidth',2.0,'HandleVisibility','off')
    end
    clear x f
end

% Create discharge_Wh vs. distance plot
figure
plot(Dataset{1,1}.cumX,Dataset{1,1}.discharged_Wh,'-k','LineWidth',1.0,'DisplayName',PXKfname(1,1))
grid on
xlabel('Distance (m)')
ylabel('Energy Discharged (Wh)')
ax = gca;
ax.FontSize = 20;
legend('Location','southeast')
fig = gcf;
fig.Position = [40 40 1000 800];
hold on
% Conduct linear fit to find discharge per unit length
pWh_dist = zeros(m,2);
pWh_dist(1,:) = polyfit(Dataset{1,1}.cumX(ndg:end-ndg),Dataset{1,1}.discharged_Wh(ndg:end-ndg),1);
x = linspace(Dataset{1,1}.cumX(ndg),Dataset{1,1}.cumX(end-ndg));
f = polyval(pWh_dist(1,:),x);
plot(x,f,':','Color','#666666','LineWidth',2.0,'HandleVisibility','off')
clear x f
for i = 2:m
    plot(Dataset{i,1}.cumX,Dataset{i,1}.discharged_Wh,LS(i),'LineWidth',1.0,'DisplayName',PXKfname(i,1))
    pWh_dist(i,:) = polyfit(Dataset{i,1}.cumX(ndg:end-ndg),Dataset{i,1}.discharged_Wh(ndg:end-ndg),1);
    x = linspace(Dataset{i,1}.cumX(ndg),Dataset{i,1}.cumX(end-ndg));
    f = polyval(pWh_dist(i,:),x);
    if i == m
        plot(x,f,':','Color','#666666','LineWidth',2.0,'DisplayName','Linear Fit')
    else
        plot(x,f,':','Color','#666666','LineWidth',2.0,'HandleVisibility','off')
    end
    clear x f
end