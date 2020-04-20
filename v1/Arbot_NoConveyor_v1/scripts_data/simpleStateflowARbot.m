
image = imread('nestlebottle-2.png');
ARbot = exampleHelperARbotPickPlace;
ARbot.changeImg(image);
ARbot.FlowChart = detector_SimpleStateflow('ARbot',ARbot);

answer = questdlg('Do you want to start the pick-and-place job now?', ...
         'Start job','Yes','No', 'No');
switch answer
    case 'Yes'
        % Trigger event to start Pick and Place in the Stateflow Chart
        % ARbot.FlowChart.startDetection;
        % simOut = sim('exampleSimulinkAndRectify')
    case 'No'
        disp('nope lel');

end
