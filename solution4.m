function [forwBackVel, leftRightVel, rotVel, finish] = solution4(pts, contacts, position, orientation)

    % State Machine (FSM)
    persistent state;
    if isempty(state),
        % the initial state of the FSM is 'init'
        state = 'init';
    end

    % initialize the robot control variables (returned by this function)
    finish = false;
    forwBackVel = 0;
    leftRightVel = 0;
    rotVel = 0;

    % get the laser contact points in sensor's coordinates
    points = [pts(1,contacts)' pts(2,contacts)'];
   
    % calculate the distances
    distances = pts(1,contacts)'.^2 + pts(2,contacts)'.^2;
    % get the closest point
    [min_value, min_index] = min(distances);

    % manage the states of FSM
    if strcmp(state, 'init')
        state = 'move_forward';
        fprintf('changing FSM state to %s\n', state);
        % save the initial position

    elseif strcmp(state, 'move_forward')
        % move straight
        forwBackVel = -3;
        leftRightVel = 0;
        rotVel = 0;

        % when close to a wall change state and move left
        if min_value <= 1
            state = 'left_move';
            fprintf('changing FSM state to %s\n', state);
        end

    elseif strcmp(state, 'left_move')

        % calculate vector to min distance
        vec_wall = points(min_index, :);

        % calculate perpendicular vector for movement
        if vec_wall(1) == 0 
            vec_per = [1 0];
        else
            vec_per = [-(vec_wall(2)/vec_wall(1)) 1];
            vec_per = 3* vec_per/norm(vec_per);
        end 

        % regulator of the distance
        
        goal_orient = atan2(vec_wall(1), -vec_wall(2));

        phi = orientation(3);
        rotVel = 5 * (goal_orient - phi);

        speed_x = cos(phi) * vec_per(1) + sin(phi) * vec_per(2);
        speed_y = -sin(phi) * vec_per(1) + cos(phi) * vec_per(2);

        % move left
        forwBackVel = speed_y;
        leftRightVel = speed_x;
%         rotVel = 0;

        
        % when there is more than 1 point equally close NOT WORKING 
        if length(min_index)>=2
            % rotate to mantain perpendicular to the wall
            fprintf('2 same points reached :)');
            % continue advancing new left
        end
        fprintf('changing FSM state to %s\n', state);
    else
        error('Unknown state %s.\n', state);
    end

end