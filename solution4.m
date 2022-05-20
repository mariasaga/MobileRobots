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
    
    % initialize function return variables
    forwBackVel = 0;
    leftRightVel = 0;
    rotVel = 0;
    finish = 0;

    % propotional gains for parallel, perpendicular, and rotational movements
    p_parallel = 1.2;
    p_perp = 2.0;
    p_orientation = 5.5;

    % absolute limits for the regulators
    par_limit = 2;
    perp_limit = 1;
    orient_limit = 10;

    % get the laser contact points in sensor's coordinates
    points = [pts(1,contacts)' pts(2,contacts)'];
    
    % calculate the distances
    distances = sum(points .^ 2, 2);
    
    % get the closest point
    [min_value, min_index] = min(distances);

    % manage the states of FSM
    if strcmp(state, 'init')
        forwBackVel = -1;
        leftRightVel = 0;
        rotVel = 0;
        
        % switch state when the distance 'dist' is reaches
        if min_value ^ 0.5 < dist
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
        % replace < by > (or vice versa) to change direction bias
        % < makes the robot go right (+ve x)
        % > makes the robot go left (-ve x)
        speed_x = cos(phi) * vec(1) + sin(phi) * vec(2);
        if speed_x < 0
            vec = -vec;
        end

        % parallel regulator
        v_para = p_parallel * vec;
        v_para(v_para > par_limit) = par_limit;
        v_para(v_para < -par_limit) = -par_limit;
        
        % perpendicular regulator
        v_perp = (norm(vec_wall) - dist) * vec_wall/norm(vec_wall);
        v_perp = p_perp * v_perp;
        v_perp(v_perp > perp_limit) = perp_limit;
        v_perp(v_perp < -perp_limit) = -perp_limit;
        
        % final global speed vector is an aggregate of the ouput of the
        % parallel and perpendicular regulators
        vec = v_para + v_perp;
        
        % orientation regulator
        % calculation of desired orientation (phi)
        goal_orient = atan2(vec_wall(1), -vec_wall(2));
        
        % avoid robot oscillation when goal orientation switches between 
        % positive and negative values near pi (e.g. 7pi/8 -> -7pi/8)
        if abs(goal_orient - phi) > pi
            if goal_orient < 0
                goal_orient = goal_orient + 2 * pi;
            else
                goal_orient = goal_orient - 2 * pi;
            end
        end

        % compute rotational velocity
        rot = p_orientation * (goal_orient - phi);
        if rot > orient_limit
            rot = orient_limit;
        elseif rot < -orient_limit
            rot = -orient_limit;
        end
    
        % global to local velocity conversion
        speed_x = cos(phi) * vec(1) + sin(phi) * vec(2);
        speed_y = -sin(phi) * vec(1) + cos(phi) * vec(2);

        % set function returns, the run is infite
        forwBackVel = speed_y;
        leftRightVel = speed_x;
        rotVel = rot;
    end
end
