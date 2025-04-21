map = binaryOccupancyMap(10, 10, 10);  % 10x resolution = 1m = 10 cells

% Place obstacles. Note: coordinates are in meters, not cell index
obstacles = [2 3; 2 4; 2 5; 3 5; 4 5];

% Mark those as occupied
setOccupancy(map, obstacles, 1);

% Inflate by 0.2 meters = 2 cells (10 cells = 1m)
inflate(map, 0.2);

figure;
show(map)
title("Occupancy Map with Obstacles and Inflation");

% Save map
save("occupancyMap.mat", "map");
