map = imread('vrep_env/exercise02.png');

[size_y, size_x] = size(map);
        
% world initial positions
x_init = 0;
y_init = 0;

goal_x = 2;
goal_y = 2;

% units conversion from world to image
x_init_map = round(size_x*((x_init-(-7.5))/(7.5-(-7.5))));
y_init_map = round(size_y*((y_init-(-7.5))/(7.5-(-7.5))));

x_goal_map = round( 100*((goal_x-(-7.5))/(7.5-(-7.5))));
y_goal_map = round( 100*((goal_y-(-7.5))/(7.5-(-7.5))));

% Wavefront Planner - Phase 1
map(map < 255) = 1;
map(map == 255) = 0;
map(y_goal_map, x_goal_map) = 2;