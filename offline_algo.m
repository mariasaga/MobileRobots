
str = 'vrep_env/exercise02';
picture = strcat(str,'.png');

map = imread(picture);

imag = map;
% figure();
% imshow(imag);

[size_y, size_x] = size(map);
        
% world initial positions
x_init = 0;
y_init = 0;

goal_x = 4;
goal_y = 4;

% units conversion from world to image
x_init_map = round(size_x*((x_init-(-7.5))/(7.5-(-7.5))));
y_init_map = round(size_y*((y_init-(-7.5))/(7.5-(-7.5))));

x_goal_map = round( 100*((goal_x-(-7.5))/(7.5-(-7.5))));
y_goal_map = round( 100*((goal_y-(-7.5))/(7.5-(-7.5))));

% Wavefront Planner - Phase 1
map(map < 255) = 1;
map(map == 255) = 0;
map(y_goal_map, x_goal_map) = 2;
map(y_init_map, x_init_map) = 0;

findy = 2;       % find value to change neighbor cell values

matrix = map;

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
            for n = (i - 1): (i + 1)
                try
                    if (matrix(m, n) == 1 || matrix(m, n) == findy || matrix(m, n)== findy - 1)
                        % hello darkness my old friend
                    else
                        matrix(m, n) = findy + 1;
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
    end
    findy = findy + 1;
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
imagesc([0 size_x], [0 size_y], imag);

hold on;

plot(solution(:, 2), solution(:, 1), 'b-*', 'linewidth', 1.5);

set(gca, 'ydir', 'normal');





% units conversion from IMAGE to WORLD
% x_world = x_image *(7.5-(-7.5))/size_x + (-7.5);
% y_world = y_image *(7.5-(-7.5))/size_y + (-7.5);


