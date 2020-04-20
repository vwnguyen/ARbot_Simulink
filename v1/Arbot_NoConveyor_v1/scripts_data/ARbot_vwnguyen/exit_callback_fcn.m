function exit_callback_fcn(obj, event, text_arg, finishFlag)

txt1 = ' event occurred at ';
txt2 = text_arg;

event_type = event.Type;
event_time = datestr(event.Data.time);
finishFlag = true;
msg = [event_type txt1 event_time];
disp(msg)
disp(txt2)