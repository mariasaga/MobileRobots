
str = 'vrep_env/exercise02';
picture = strcat(str,'.png');

map = imread(picture);

imag = map;
% figure();
% imshow(imag);

[size_y, size_x] = size(map);
        
% world initial positions
x_init = 1;
y_init = 0;

goal_x = 6;
goal_y = 6;

% units conversion from world to image
x_init_map = round(size_x*((x_init-(-7.5))/(7.5-(-7.5))));
y_init_map = round(size_y*((y_init-(-7.5))/(7.5-(-7.5))));

x_goal_map = round(size_x*((goal_x-(-7.5))/(7.5-(-7.5))));
y_goal_map = round(size_y*((goal_y-(-7.5))/(7.5-(-7.5))));

% Wavefront Planner - Phase 1
map(map < 255) = 1;
map(map == 255) = 0;
map(y_goal_map, x_goal_map) = 2;
map(y_init_map, x_init_map) = 0;


% where we have 1 we thicken the wall
num_neigh = round(0.65/15 * 100);      % number of neighbor cells to make walls
[a, b] = ind2sub(size(map), find(map == 1));

for k = 1: length(a)
    j = a(k); 
    i = b(k);
    for m = (j - num_neigh): (j + num_neigh)
        for n = (i - num_neigh): (i + num_neigh)
            try
                if (map(m,n) == 2 || (x_init_map == n && y_init_map == m) || map(m,n) == 1)
                else
                    % thicken the wall for the neighbors
                    map(m,n) = 1; 
                end
            catch ME
                if strcmp(ME.identifier, 'MATLAB:badsubscript')
                else
                    throw(ME)
                end
            end
        end  
    end   
end


findy = 2;       % find value to change 4 neighbor cell values

matrix = map;



map(map == 1) = 20;
map(map == 2) = 125;
map(map == 0) = 255;
imag = map;
figure();
imshow(imag);

% while the initial position has not been reasigned a value
while matrix(y_init_map, x_init_map) == 0
    
    [a, b] = ind2sub(size(matrix), find(matrix == findy));

    for k = 1: length(a)
        j = a(k); 
        i = b(k); 
        if matrix(j, i) == 1
            continue
        end
        for m = (j - 1): (j + 1)
            try
                if (matrix(m, i) == 1 || matrix(m, i) == findy || matrix(m, i) == findy - 1)
                else
                    matrix(m, i) = findy + 1;
                end
            catch ME
                if strcmp(ME.identifier, 'MATLAB:badsubscript')
                else
                    throw(ME)
                end
            end
        end
        for n = i - 1: i + 1
            try 
                if (matrix(j, n) == 1 || matrix(j, n) == findy || matrix(j, n)== findy - 1)
                else
                    matrix(j, n) = findy + 1;
                end
            catch ME
                if strcmp(ME.identifier, 'MATLAB:badsubscript')
                else
                    throw(ME)
                end
            end
        end
    end
    findy = findy + 1
end


% Wavefront Planner - Phase 2

goal_map = [y_goal_map, x_goal_map];
solution = [];

current = [y_init_map, x_init_map];
while current(1) ~= goal_map(1) || current(2) ~= goal_map(2)
    j = current(1);
    i = current(2);
    min_value = matrix(current(1), current(2));
    min_index = current;

    for m = (j + 1): -1: (j - 1)
        for n = (i + 1): -1: (i - 1)
            try
                if matrix(m, n) > 1
                    if (matrix(m, n) < min_value)
                        min_value = matrix(m, n);
                        min_index = [m, n];
                    end
                end
            catch ME
                if strcmp(ME.identifier, 'MATLAB:badsubscript')
                    % hello darkness my old friend
                else
                    throw(ME)
                end
            end
        end  
    end
    solution = [solution; min_index];
    current = min_index;
end


% plotting
figure;
imagesc([0 size_x], [0 size_y], imag);

hold on;

plot(solution(:, 2), solution(:, 1), 'b-*', 'linewidth', 1.5);

set(gca, 'ydir', 'normal');

% units conversion from IMAGE to WORLD
x_world = solution(:, 2) .* (7.5-(-7.5))/size_x + (-7.5);
y_world = solution(:, 1) .* (7.5-(-7.5))/size_y + (-7.5);
