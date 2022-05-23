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

    if length(varargin) < 2
        disp("Expected an input goal position");
        finish = 1;
        return;
    else
        d = varargin{1};
        goal_x = d(1);
        goal_y = d(2);
        map2 = varargin{2};
    end


    % manage the states of FSM
    if strcmp(state, 'init')

        % Path Planning 
        
        % check map exists
        Directory = fullfile ('C:','emor','ws_emor','emor_trs','youbot', 'vrep_env', map2);
        Images = dir(fullfile(Directory,'*.png'));
        Names = {Images.name}';
        for k=1: numel(Names)
               try
                I=imread(fullfile(Directory,Names{k})); 
              catch
                error( ...)
                end
               end
        end

        % read the map 
        map = imread('vrep_env/map2.png');  % we have to add the .png? check if it exists

        % get size of map
        [size_y, size_x] = size(map);
        
        % create a matrix where to work 
        matrix = zeros(size_y, size_x);

        % world initial positions
        x_init = position(1);
        y_init = position(2);

        % units conversion from world to image
        x_init_map=round( size_x*((x_init-(-7.5))/(7.5-(-7.5))));
        y_init_map=round( size_y*((y_init-(-7.5))/(7.5-(-7.5))));

        x_goal_map = round( 100*((goal_x-(-7.5))/(7.5-(-7.5))));
        y_goal_map = round( 100*((goal_y-(-7.5))/(7.5-(-7.5))));


        % Wavefront Planner - Phase 1
        for i = 1: size_x
            for j = 1: size_y
                % if the cell is free
                if map(j, i) == 255
                    matrix(j,i) = 0;
                % if the cell is occupied (obstacles)
                elseif map(j,i) == 0
                    matrix(j,i) = 1;
                % if the cell is the goal cell put a 2
                elseif i == x_goal_map && j == y_goal_map
                    matrix(j,i) = 2;
                % if unclassified, obstacle
                else 
                    matrix(j,i) = 1; 
                end
            end
        end
        
        find = 2;       % find value to change neighbor cell values

        % while the initial position has not been reasigned a value
        while matrix (y_init_map, x_init_map) == 0
            for i = 1: size_x
                for j = 1: size_y
                    % check if we are in the found value
                     if matrix(j,i) == find

                        % check not a corner

                        % check neighbors not find

                        matrix(j+1,i) = find + 1;
                        matrix(j-1, i) = find + 1;
                        matrix(j, i+1) = find + 1;
                        matrix(j, i-1) = find + 1;
                        matrix(j+1, i+1) = find + 1;
                        matrix(j+1, i-1) = find + 1;
                        matrix(j-1, i+1) = find + 1;
                        matrix(j-1, i-1) = find + 1;


                        find = find + 1; % update value to find
                     end

                end
            end
        end

        % Wavefront Planner - Phase 2
        for i = 1: size_x
            for j = 1: size_y
                

            end
        end


        % units conversion from IMAGE to WORLD
        x_world = x_image *(7.5-(-7.5))/size_x + (-7.5);
        y_world = y_image *(7.5-(-7.5))/size_y + (-7.5);
        

    elseif strcmp(state, 'move')

        % Follow path created



        % check if goal position reached

    end




end
