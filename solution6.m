function [forwBackVel, leftRightVel, rotVel, finish] = solution6(pts, contacts, position, orientation, varargin)

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

    persistent x_world;
    persistent y_world;
    persistent curr;

    % tolerance of final destination
    tolerances = [0.05 0.05];

    % for propotional regulator
    gain_P= [12 12];          % gains for x, y
    u_max = [5 5];            % positive saturation values for speeds of x, y
    u_min = [-5 -5];          % negative saturation values for speeds of x, y

    if length(varargin) < 2
        disp("Expected an input goal position");
        finish = 1;
        return;
    else
        d = varargin{1};
        goal_x = d(1);
        goal_y = d(2);
        str = varargin{2};
    end

    % manage the states of FSM
    if strcmp(state, 'init')

        % read image
        str = strcat('vrep_env/', str);
        picture = strcat(str, '.png');
        matrix = imread(picture);
        map = matrix;

        % get size of matrix
        [size_y, size_x] = size(matrix);

        % world initial positions
        x_init = position(1);
        y_init = position(2);

        % units conversion from world to image
        x_init_matrix = round(size_x*((x_init-(-7.5))/(7.5-(-7.5))));
        y_init_matrix = round(size_y*((y_init-(-7.5))/(7.5-(-7.5))));
        x_goal_matrix = round(size_x*((goal_x-(-7.5))/(7.5-(-7.5))));
        y_goal_matrix = round(size_y*((goal_y-(-7.5))/(7.5-(-7.5))));
        
        % Wavefront Planner - Phase 1
        matrix(matrix < 255) = 1;
        matrix(matrix == 255) = 0;
        matrix(y_goal_matrix, x_goal_matrix) = 2;
        matrix(y_init_matrix, x_init_matrix) = 0;
        
        % where we have 1 we thicken the wall
        num_neigh = round(0.5/15 * 100);      % number of neighbor cells to make walls
        [a, b] = ind2sub(size(matrix), find(matrix == 1));
        
        for k = 1: length(a)
            j = a(k); 
            i = b(k); 
            for m = (j - num_neigh): (j + num_neigh)
                for n = (i - num_neigh): (i + num_neigh)
                    if (m > 0 && n > 0 && m <= size_y && n <= size_x)
                        if (matrix(m, n) == 2 || (x_init_matrix == n && y_init_matrix == m) || matrix(m, n) == 1)
                        else
                            % thicken the wall for the neighbors   
                            matrix(m,n) = 1; 
                        end
                    end
                end  
            end   
        end
        
        findy = 2;       % find value to change 4 neighbor cell values

        % while the initial position has not been reasigned a value
        while matrix(y_init_matrix, x_init_matrix) == 0
            
            [a, b] = ind2sub(size(matrix), find(matrix == findy));
        
            for k = 1: length(a)
                j = a(k); 
                i = b(k); 
                if matrix(j, i) == 1
                    continue
                end
                for m = (j - 1): (j + 1)
                    if (m > 0 && m <= size_y)
                        if (matrix(m, i) == 1 || matrix(m, i) == findy || matrix(m, i) == findy - 1)
                        else
                            matrix(m, i) = findy + 1;
                        end
                    end
                end
                for n = (i - 1): (i + 1)
                    if (n > 0 && n <= size_x)
                        if (matrix(j, n) == 1 || matrix(j, n) == findy || matrix(j, n) == findy - 1)
                        else
                            matrix(j, n) = findy + 1;
                        end
                    end
                end
            end
            findy = findy + 1;
        end
             
        % Wavefront Planner - Phase 2
        goal_matrix = [y_goal_matrix, x_goal_matrix];
        solution = [];
        
        current = [y_init_matrix, x_init_matrix];
        while current(1) ~= goal_matrix(1) || current(2) ~= goal_matrix(2)
            j = current(1);
            i = current(2);
            min_value = matrix(current(1), current(2));
            min_index = current;
        
            for m = (j + 1): -1: (j - 1)
                for n = (i + 1): -1: (i - 1)
                    if (m > 0 && n > 0 && m <= size_y && n <= size_x)
                        if matrix(m, n) > 1
                            if (matrix(m, n) < min_value)
                                min_value = matrix(m, n);
                                min_index = [m, n];
                            end
                        end
                    end
                end  
            end
            solution = [solution; min_index];
            current = min_index;
        end

        % plotting
        figure;
        imagesc([0 size_x], [0 size_y], map);
        hold on;
        plot(solution(:, 2), solution(:, 1), 'b-*', 'linewidth', 1.5);
        set(gca, 'ydir', 'normal');

        % units conversion from IMAGE to WORLD
        x_world = solution(:, 2) .* (7.5-(-7.5))/size_x + (-7.5);
        y_world = solution(:, 1) .* (7.5-(-7.5))/size_y + (-7.5);      
        
        curr = 1;
        state = 'move';

    elseif strcmp(state, 'move')
        
        % Follow path created
        dest = [x_world(curr), y_world(curr)];
        u = zeros(2, 1);

        % proportional regulator for x, y, phi
        for i = 1: 2
            measured = position(i);
            errors = dest(i) - measured;
            u(i) = gain_P(i) * errors;
            if u(i) > u_max(i)
                u(i) = u_max(i);
            elseif u(i) < u_min(i)
                u(i) = u_min(i);
            end
        end

        % changing global velocities to local
        phi = orientation(3);
        speed_x = cos(phi) * u(1) + sin(phi) * u(2);
        speed_y = -sin(phi) * u(1) + cos(phi) * u(2);

        % setting speeds
        forwBackVel = speed_y;
        leftRightVel = speed_x;

        % criteria to advance current division
        if curr < length(x_world)
            if abs(position(1) - dest(1)) <= 0.15 && abs(position(2) - dest(2)) <= 0.15 
                curr = curr + 1;
            end
        end
        
        % check if goal position reached
        if abs(position(1) - goal_x) <= tolerances(1) && abs(position(2) - goal_y) <= tolerances(2)
            fprintf('changing FSM state to %s\n', state);
            finish = 1;
        end
    end
end
