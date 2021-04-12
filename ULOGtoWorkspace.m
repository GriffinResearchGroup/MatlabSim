%This script cycles through a recorded ULOG file, saving the contents of
%each recorded topic to a variable in the workspace.

ulogOBJ = ulogreader('20_47_55.ulg'); %Specify the ULOG file

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