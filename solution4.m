function [forwBackVel, leftRightVel, rotVel, finish] = solution4(pts, contacts, position, orientation)

    % State Machine (FSM)
    persistent state;
    if isempty(state)
        % the initial state of the FSM is 'init'
        state = 'init';
    end

    dist = 1;
    p_parallel = 2;

    % initialize the robot control variables (returned by this function)
    finish = false;
    forwBackVel = 0;
    leftRightVel = 0;
    rotVel = 0.1;

    % get the laser contact points in sensor's coordinates
     points = [pts(1,contacts)' pts(2,contacts)'];
    % calculate the distances
    distances = (pts(1,contacts)'.^2 + pts(2,contacts)'.^2).^0.5;
    % get the closest point
    [min_value, min_index] = min(distances(:));

    % manage the states of FSM
    if strcmp(state, 'init')
        forwBackVel = -3;
        leftRightVel = 0;
        rotVel = 0;
        
        if min_value < dist
            state = 'move';
        end

    elseif strcmp(state, 'move')

        % calculate vector to min distance
        phi = orientation(3);
        vec_wall = points(min_index, :);
        x = cos(phi) * vec_wall(1) - sin(phi) * vec_wall(2);
        y = sin(phi) * vec_wall(1) + cos(phi) * vec_wall(2);
        vec_wall = [x y];

        % calculate perpendicular vector for movement
        if vec_wall(1) == 0 
            vec = [1 0];
        else
            vec = [-vec_wall(2)/vec_wall(1) 1];
            vec = vec/norm(vec);
        end 

        speed_x = cos(phi) * vec(1) + sin(phi) * vec(2);

        if speed_x < 0
            vec = -vec;
        end
        
        
        % distance regulator perp
        (norm(vec_wall) - dist) * vec_wall;


        vec = P * dist * vec;

        goal_orient = atan2(vec_wall(1), -vec_wall(2));
        rotVel = 10 * (goal_orient - phi);

        speed_x = cos(phi) * vec(1) + sin(phi) * vec(2);
        speed_y = -sin(phi) * vec(1) + cos(phi) * vec(2);
        
        forwBackVel = speed_y;
        leftRightVel = speed_x;

    else
        error('Unknown state %s.\n', state);
    end

end
