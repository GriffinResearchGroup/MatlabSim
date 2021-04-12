clear
clc
close all

object = stlread('TestPlane.stl'); %Specify STL object for visualization

data = readmatrix('flight_test3.csv'); %Specify CSV vicon data
ViconFrameRate = 60;

%video setup
video = VideoWriter('ViconSTL_Recent.avi');
video.FrameRate = 30;
%skipWrite make the saved video realtime, by copying or skipping frames
skipWrite = (ViconFrameRate/video.FrameRate)*(ViconFrameRate>=video.FrameRate)...
    +(video.FrameRate/ViconFrameRate)*(ViconFrameRate<video.FrameRate);
open(video)

%Process Data
FR = data(:,1);
validFR = FR(~isnan(data(:,7)));
quat = quaternion(data(:,6),data(:,3),data(:,4),data(:,5));
Tvec = [data(:,7),data(:,8),data(:,9)];
STLpoints = object.Points;
connected = object.ConnectivityList;

%Veiw Settings
ZoomOutObj = 200; %mm
Xbounds = [min(Tvec(:,1))-min(STLpoints(:,1)),max(Tvec(:,1))+max(STLpoints(:,1))];
Ybounds = [min(Tvec(:,2))-min(STLpoints(:,2)),max(Tvec(:,2))+max(STLpoints(:,2))];
Zbounds = [min(Tvec(:,3))-min(STLpoints(:,3)),max(Tvec(:,3))+max(STLpoints(:,3))];
f = figure;
u = f.WindowState;
f.WindowState = 'maximized';

%Visualization section
w = 1;
for i = 1:length(validFR)
    frame = validFR(i);
    rotatedObjectPoints = rotatepoint(quat(frame),STLpoints);
    transAndRotPoints = rotatedObjectPoints + Tvec(frame,:);
    
    subplot(1,2,1)
    plot3(Tvec(:,1),Tvec(:,2),Tvec(:,3),'-o','MarkerIndices',frame)
    xlim([Xbounds(1),Xbounds(2)])
    ylim([Ybounds(1),Ybounds(2)])
    zlim([Zbounds(1),Zbounds(2)])
    axis equal
    grid on
    xlabel('X')
    ylabel('Y')
    zlabel('Z')

    subplot(1,2,2)
    plot3(Tvec(:,1),Tvec(:,2),Tvec(:,3))
    trisurf(connected,transAndRotPoints(:,1),transAndRotPoints(:,2),transAndRotPoints(:,3),...
        'FaceColor',[1,1,1],'EdgeColor','k');
    xlim([min(transAndRotPoints(:,1))-ZoomOutObj,max(transAndRotPoints(:,1))+ZoomOutObj])
    ylim([min(transAndRotPoints(:,2))-ZoomOutObj,max(transAndRotPoints(:,2))+ZoomOutObj])
    zlim([min(transAndRotPoints(:,3))-ZoomOutObj,max(transAndRotPoints(:,3))+ZoomOutObj])
    %axis equal
    grid off
    xlabel('X')
    ylabel('Y')
    zlabel('Z')

    %Video Writing Section
    if w>=skipWrite && (ViconFrameRate>=video.FrameRate)
       writeVideo(video, getframe(f)) 
       w = 1;
    elseif (ViconFrameRate<video.FrameRate)
        for p = 1:round(skipWrite)
            writeVideo(video, getframe(f)) 
        end
    end
    w = w+1;
    disp(frame)
end
close(video)









