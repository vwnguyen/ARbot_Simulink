% t = timer('TimerFcn', 'stat=false; disp(''Timer!'')',...
%     'StartDelay',10);
% start(t)
% stat=true;
% while(stat==true)
%     disp('.')
%     pause(1)
% end
% 
% delete(t) % Always delete timer objects after using them.
queue = [ 2 4 6 7 ]; 
finishFlag = false;
t = timer('Period', 1, 'TasksToExecute', 3, ...
          'ExecutionMode', 'fixedRate');      
t.StartFcn = { @my_callback_fcn, 'My start message'};
t.StopFcn = { @exit_callback_fcn, 'My stop message', finishFlag};
t.TimerFcn = { @update_queue, queue } ;
start(t)

while finishFlag == false
    % do nothing
end
    
% get rid of any running timer objects
delete(timerfindall)