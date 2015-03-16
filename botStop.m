%%simple function just to stop to bot
function botStop()

disp('Stopping!');
wb_differential_wheels_set_speed(0,0);
wb_robot_step(64);
pause(1);   
   
end
 
