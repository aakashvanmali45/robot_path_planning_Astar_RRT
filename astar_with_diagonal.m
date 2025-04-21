clc;
clear;

% Load occupancy map
load('occupancyMap.mat', 'map');
grid = occupancyMatrix(map);  % 1 = obstacle, 0 = free

% Convert start/goal from meters to grid indices
start_world = [1, 1];
goal_world = [3, 6];

start = world2grid(map, start_world);
goal = world2grid(map, goal_world);

% Safety check
if grid(start(1), start(2)) == 1 || grid(goal(1), goal(2)) == 1
    error('Start or goal is on an obstacle!');
end

% Initialize open and closed sets
openList = [start, 0, heuristic(start, goal), 0 + heuristic(start, goal), -1, -1];
closedList = [];

% Directions: 8-connectivity (up, down, left, right + diagonals)
dirs = [ 0  1;  0 -1;  1  0; -1  0;  % straight
         1  1;  1 -1; -1  1; -1 -1]; % diagonals

found = false;

while ~isempty(openList)
    % Get node with lowest f
    [~, idx] = min(openList(:,5));
    current = openList(idx, :);
    openList(idx, :) = [];

    if isequal(current(1:2), goal)
        found = true;
        goalNode = current;
        break;
    end

    closedList = [closedList; current];

    for i = 1:size(dirs,1)
        neighbor = current(1:2) + dirs(i,:);
        x = neighbor(1); y = neighbor(2);

        % Check bounds
        if x < 1 || x > size(grid,1) || y < 1 || y > size(grid,2)
            continue;
        end

        % Skip if obstacle
        if grid(x, y) == 1
            continue;
        end

        % Skip if already in closed list
        if any(all(closedList(:,1:2) == neighbor, 2))
            continue;
        end

        % Diagonal step cost √2, straight step cost 1
        if abs(dirs(i,1)) + abs(dirs(i,2)) == 2
            stepCost = sqrt(2);
        else
            stepCost = 1;
        end

        g = current(3) + stepCost;
        h = heuristic(neighbor, goal);
        f = g + h;

        existingIdx = find(all(openList(:,1:2) == neighbor, 2));
        if ~isempty(existingIdx)
            if openList(existingIdx, 5) > f
                openList(existingIdx, :) = [x, y, g, h, f, current(1), current(2)];
            end
        else
            openList = [openList; x, y, g, h, f, current(1), current(2)];
        end
    end
end

% Backtrack path
if found
    path = [goalNode(1), goalNode(2)];
    parent = goalNode(6:7);
    while ~isequal(parent, -1*ones(1,2))
        path = [parent; path];
        idx = find(all(closedList(:,1:2) == parent, 2));
        parent = closedList(idx, 6:7);
    end

    % Convert path to world coordinates
    pathWorld = grid2world(map, path);

    % Plot
    figure;
    show(map); hold on;
    plot(pathWorld(:,1), pathWorld(:,2), 'r-', 'LineWidth', 2);
    plot(start_world(1), start_world(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    plot(goal_world(1), goal_world(2), 'bx', 'MarkerSize', 10, 'LineWidth', 2);
    legend('Path', 'Start', 'Goal');
    title("A* Path Planning with Diagonal Movement");
else
    disp("No path found.");
end

% Heuristic function (Manhattan or Diagonal)
function h = heuristic(node, goal)
    dx = abs(goal(1) - node(1));
    dy = abs(goal(2) - node(2));
    % Octile distance (used for diagonal movement)
    h = dx + dy + (sqrt(2) - 2) * min(dx, dy);
end
