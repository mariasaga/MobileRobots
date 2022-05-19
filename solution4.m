function [forwBackVel, leftRightVel, rotVel, finish] = solution4(pts, contacts, position, orientation, varargin)

    % State Machine (FSM)
    persistent state;
    if isempty(state)
        % the initial state of the FSM is 'init'
        state = 'init';
    end

    if length(varargin) < 1
        dist = 0.75;
    else
        dist = varargin{1};
    end
    
    % propotional gains for parallel and perpendicular movements
    p_parallel = 1;
    p_perp = 2;

    % get the laser contact points in sensor's coordinates
    points = [pts(1,contacts)' pts(2,contacts)'];
    
    % calculate the distances
    distances = pts(1,contacts)'.^2 + pts(2,contacts)'.^2;
    
    % get the closest point
    [min_value, min_index] = min(distances);

    % manage the states of FSM
    if strcmp(state, 'init')
        forwBackVel = -1;
        leftRightVel = 0;
        rotVel = 0;
        
        if min_value < dist ^ 2
            state = 'move';
        end

    elseif strcmp(state, 'move')
        % calculate vector with min distance in global coordinates
        phi = orientation(3);
        vec_wall = points(min_index, :);

        % sensor coordinates to global
        x = cos(phi) * vec_wall(1) - sin(phi) * vec_wall(2);
        y = sin(phi) * vec_wall(1) + cos(phi) * vec_wall(2);
        vec_wall = [x y];

        % calculate perpendicular vector for regulator to maintain distance
        if vec_wall(1) == 0 
            vec = [1 0];
        else
            vec = [-vec_wall(2)/vec_wall(1) 1];
            vec = vec/norm(vec);
        end 

        % ensure that the robot only moves in one direction.
        % replace < by > (or vice versa) to change direction bise
        % < makes the robot go right (+ve x)
        % > makes the robot go left (-ve x)
        speed_x = cos(phi) * vec(1) + sin(phi) * vec(2);
        if speed_x < 0
            vec = -vec;
        end

        % error in constant distance from wall
        v_perp = (norm(vec_wall) - dist) * vec_wall/norm(vec_wall);
        
        vec = p_parallel * vec + p_perp * v_perp;
        
        goal_orient = atan2(vec_wall(1), -vec_wall(2));
        if abs(goal_orient - phi) > pi
            if goal_orient < 0
                goal_orient = goal_orient + 2 * pi;
            else
                goal_orient = goal_orient - 2 * pi;
            end
        end
        rotVel = 5 * (goal_orient - phi);

        speed_x = cos(phi) * vec(1) + sin(phi) * vec(2);
        speed_y = -sin(phi) * vec(1) + cos(phi) * vec(2);

        forwBackVel = speed_y;
        leftRightVel = speed_x;
    end
    
    finish = 0;

end
