%This script cycles through a recorded ULOG file, saving the contents of
%each recorded topic to a variable in the workspace.

%Clear all previous variables
clear;
clc;
% Prompt user to select .csv file to work with
[FileName,PathName,FilterIndex] = uigetfile('*.ulg');
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
plot(x,y)