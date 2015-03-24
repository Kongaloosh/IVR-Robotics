function move(arg)

    disp('Starting now!');

    switch arg
        case 'forwardFast'
            moveFastForward();
        case 'forward'
            moveForward();
    end

end

function moveFastForward()
    disp('Moving forward');
    wb_differential_wheels_set_speed(6,6);
  	wb_robot_step(64);
end

function moveForward()
  	wb_differential_wheels_set_speed(3,3);
  	wb_robot_step(64);
    disp('forward');
end
