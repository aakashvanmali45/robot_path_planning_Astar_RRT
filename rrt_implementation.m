clc; clear;

load("occupancyMap.mat","map");
grid = occupancyMatrix(map);
[rows, cols] = size(grid);

start = [1, 1];
goal = [6, 3];

max_iter = 4;
step_size = 1.5;
goal_thresh = 1;

if grid(start(1), start(2)) == 1 || grid(goal(1), goal(2)) == 1
    error("Start or goal is on an obstacle!");
end

nodes(1).pos = start;
nodes(1).parent = 0;

figure;
show(map); hold on; 
plot(start(2), start(1), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(goal(2), goal(1), 'bx', 'MarkerSize', 10, 'LineWidth', 2);
title("RRT Path Planning");

for i = 1:max_iter
    rand_point = [randi(rows), randi(cols)];

    dists = cellfun(@(p) norm(p - rand_point), {nodes.pos}); %distance between every node in rt to the new random node
    [~, idx] = min(dists);
    nearest = nodes(idx).pos;

    dir = rand_point - nearest;
    dir = dir / norm(dir);
    new_point = round(nearest + step_size * dir);

    
    if ~isCollisionFree(nearest, new_point, grid)
        continue;
    end


    if any(new_point < 1) || new_point(1) > rows || new_point(2) > cols
        continue;
    end

    nodes(end + 1).pos = new_point;
    nodes(end).parent = idx;

    plot([nearest(2), new_point(2)], [nearest(1), new_point(1)], 'b');
    drawnow limitrate;

    if norm(new_point - goal) < goal_thresh
        disp("Goal reached!");
        nodes(end+1).pos = goal;
        nodes(end).parent = numel(nodes) - 1;
        break;
    end
end

path = goal;
parent = nodes(end).parent;

while parent ~= 0
    path = [nodes(parent).pos; path];
    parent = nodes(parent).parent;
end

plot(path(:,2), path(:,1), 'r-', 'LineWidth', 2);
legend("Start", "Goal", "RRT Tree", "Final Path");

function free = isCollisionFree(p1, p2, grid)
    % Bresenham-style collision check between two points
    points = round([linspace(p1(1), p2(1), 100); linspace(p1(2), p2(2), 100)])';
    
    for i = 1:size(points, 1)
        r = points(i, 1);
        c = points(i, 2);
        if r < 1 || r > size(grid, 1) || c < 1 || c > size(grid, 2)
            free = false;
            return;
        end
        if grid(r, c) == 1
            free = false;
            return;
        end
    end
    free = true;
end


rrt_path = flip(path, 2); % Convert [row,col] -> [x,y]
assignin('base', 'rrt_path', rrt_path);


