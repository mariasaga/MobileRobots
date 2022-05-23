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
        
        % read the map 
        map = imread('vrep_env/map2.png');  % we have to add the .png? check if it exists

        % get size of map
        [size_y, size_x] = size(map);


        % units conversion from image to world
        x_map=round( 100*((x_world-(-7.5))/(7.5-(-7.5))));
        y_map=round( 200*((x_world-(-7.5))/(7.5-(-7.5))));

    elseif strcmp(state, 'move')

        % Follow path created



        % check if goal position reached

    end




end
