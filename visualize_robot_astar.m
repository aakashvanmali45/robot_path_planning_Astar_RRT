clc;
clear;

% Load occupancy map
load('occupancyMap.mat', 'map');
grid = occupancyMatrix(map);  % 1 = obstacle, 0 = free

% Convert start/goal from meters to grid indices
start_world = [1, 1];
goal_world = [3, 6];

% Convert to grid coordinates
start = world2grid(map, start_world);
goal = world2grid(map, goal_world);

% Safety check
if grid(start(1), start(2)) == 1 || grid(goal(1), goal(2)) == 1
    error('Start or goal is on an obstacle!');
end

% Initialize open and closed sets
openList = [start, 0, heuristic(start, goal), 0 + heuristic(start, goal), -1, -1]; % [x, y, g, h, f, px, py]
closedList = [];

% Directions: up, down, left, right (no diagonal)
dirs = [ 0  1;  0 -1;  1  0; -1  0];

found = false;

while ~isempty(openList)
    % Get node with lowest f
    [~, idx] = min(openList(:,5));
    current = openList(idx, :);
    openList(idx, :) = []; % Remove from open

    % Check if goal reached
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

        g = current(3) + 1;
        h = heuristic(neighbor, goal);
        f = g + h;

        % Check if in open list with higher cost
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
    title("A* Path Planning without Diagonal");

 
    robotPos = pathWorld(1, :); 
    hRobot = plot(robotPos(1), robotPos(2), 'bo', 'MarkerSize', 2, 'LineWidth', 2);
    
    for i = 2:size(pathWorld, 1)
        robotPos = pathWorld(i, :);
        set(hRobot, 'XData', robotPos(1), 'YData', robotPos(2)); 
        drawnow;  
        pause(0.2);  
    end
else
    disp("No path found.");
end


function h = heuristic(node, goal)
    h = abs(goal(1) - node(1)) + abs(goal(2) - node(2)); % Manhattan
end
