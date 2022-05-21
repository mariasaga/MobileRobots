function [forwBackVel, leftRightVel, rotVel, finish] = solution5(pts, contacts, position, orientation, varargin)

    % State Machine (FSM)
    persistent state;
    if isempty(state)
        % the initial state of the FSM is 'init'
        state = 'init';
    end
   
    % initialize function return variables
    forwBackVel = 0;
    leftRightVel = 0;
    rotVel = 0;
    finish = 0;

    if length(varargin) < 1
        disp("Expected an input goal position");
        finish = 1;
        return;
    else
        d = varargin{1};
        goal_x = d(1);
        goal_y = d(2);
    end
    
    % line characteristics
    persistent m;               % gradient of line
    persistent c;               % y-intercept of line
    persistent dist_to_goal;    % distance to goal
    persistent x_s;
    persistent y_s;
    persistent p_limit;

    tolerance = 0.02;           % tolerance in m 
    part_len = 0.01;
    angle_tol = 3;              % angle tolerance in degrees
    obj_detect_dist = 0.2;

    % regulator constants for moving in a straight line
    p_line = 3;
    limit = 5;

    % propotional gains for parallel, perpendicular, and rotational movements
    p_parallel = 1.5;
    p_perp = 2.0;
    p_orientation = 5.5;

    % absolute limits for the regulators
    par_limit = 2;
    perp_limit = 1;
    orient_limit = 10;

    % get the laser contact points in sensor coordinates
    points = [pts(1,contacts)' pts(2,contacts)'];
    
    % calculate the squared distances
    distances = sum(points .^ 2, 2);
    
    % get the closest point
    [min_value, min_index] = min(distances);

    % manage the states of FSM
    if strcmp(state, 'init')
        
        % calculate line parameters
        x0 = position(1);
        y0 = position(2);

        m = (goal_y - y0)/(goal_x - x0);
        c = y0 - m * x0;

        dist_to_goal = sqrt((goal_x - x0) ^ 2 + (goal_y - y0) ^ 2);

        if abs(m) < 1
            p_limit = [limit limit * m];
        else
            p_limit = [limit/m limit];
        end

        state = 'rotate_to_goal';

    elseif strcmp(state, 'rotate_to_goal')

        phi = orientation(3);
        goal_orient = atan2(goal_x - position(1), position(2) - goal_y);
        
        if abs(phi - goal_orient) < angle_tol * pi/180
            state = 'move_to_goal';
        end
        
        % change orientation to one that is needed, if needed
        error = goal_orient - phi;
        rotVel = p_orientation * error;
        if rotVel > orient_limit
            rotVel = orient_limit;
        elseif rotVel < -orient_limit
            rotVel = -orient_limit;
        end

    elseif strcmp(state, 'move_to_goal')
        
        if abs(position(1) - goal_x) < tolerance && ...
               abs(position(2) - goal_y) < tolerance
            disp("Goal reached!")
            finish = 1;
            return;
        end
        
        if min_value ^ 0.5 < obj_detect_dist
            dist_to_goal = sqrt((goal_x - position(1)) ^ 2 + (goal_y - position(2)) ^ 2);
            if dist_to_goal >= obj_detect_dist
                state = 'avoid_obstacle';
            end
        end

        % regulators for movement
        u = zeros(1, 2);
        goals = [goal_x goal_y];

        for i = 1: 2
            error = goals(i) - position(i);
            u(i) = p_line * error;
            if u(i) > p_limit(i)
                u(i) = p_limit(i);
            elseif u(i) < -p_limit(i)
                u(i) = -p_limit(i);
            end
        end

        % changing global velocities to local
        phi = orientation(3);
        speed_x = cos(phi) * u(1) + sin(phi) * u(2);
        speed_y = -sin(phi) * u(1) + cos(phi) * u(2);

        % setting speeds
        forwBackVel = speed_y;
        leftRightVel = speed_x;
        rotVel = 0;

    elseif strcmp(state, 'avoid_obstacle')


        % if point on line
        y_line = m * position(1) + c;
        if abs(y_line - position(2)) < tolerance
            disp("Point on line");

            % if new distance to goal less than previously saved distance
            dist = sqrt((goal_x - position(1)) ^ 2 + (goal_y - position(2)) ^ 2);
            if dist <= dist_to_goal
                state = 'rotate_to_goal';
            end
        end

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
        if speed_x > 0
            vec = -vec;
        end

        % parallel regulator
        v_para = p_parallel * vec;
        v_para(v_para > par_limit) = par_limit;
        v_para(v_para < -par_limit) = -par_limit;
        
        % perpendicular regulator
        v_perp = (norm(vec_wall) - obj_detect_dist) * vec_wall/norm(vec_wall);
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
