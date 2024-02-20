% ======
% ROB521_assignment1.m
% ======
%
% This assignment will introduce you to the idea of motion planning for  
% holonomic robots that can move in any direction and change direction of 
% motion instantaneously.  Although unrealistic, it can work quite well for
% complex large scale planning.  You will generate mazes to plan through 
% and employ the PRM algorithm presented in lecture as well as any 
% variations you can invent in the later sections.
% 
% There are three questions to complete (5 marks each):
%
%    Question 1: implement the PRM algorithm to construct a graph
%    connecting start to finish nodes.
%    Question 2: find the shortest path over the graph by implementing the
%    Dijkstra's or A* algorithm.
%    Question 3: identify sampling, connection or collision checking 
%    strategies that can reduce runtime for mazes.
%
% Fill in the required sections of this script with your code, run it to
% generate the requested plots, then paste the plots into a short report
% that includes a few comments about what you've observed.  Append your
% version of this script to the report.  Hand in the report as a PDF file.
%
% requires: basic Matlab, 
%
% S L Waslander, January 2022
%
clear; close all; clc;

% set random seed for repeatability if desired
% rng(1);

% % ==========================
% % Maze Generation
% % ==========================
% %
% % The maze function returns a map object with all of the edges in the maze.
% % Each row of the map structure draws a single line of the maze.  The
% % function returns the lines with coordinates [x1 y1 x2 y2].
% % Bottom left corner of maze is [0.5 0.5], 
% % Top right corner is [col+0.5 row+0.5]
% %
% 
% row = 5; % Maze rows
% col = 7; % Maze columns
% map = maze(row,col); % Creates the maze
% start = [0.5, 1.0]; % Start at the bottom left
% finish = [col+0.5, row]; % Finish at the top right
% 
% h = figure(1);clf; hold on;
% plot(start(1), start(2),'go')
% plot(finish(1), finish(2),'rx')
% show_maze(map,row,col,h); % Draws the maze
% drawnow;
% 
% 
% 
% % ======================================================
% % Question 1: construct a PRM connecting start and finish
% % ======================================================
% %
% % Using 500 samples, construct a PRM graph whose milestones stay at least 
% % 0.1 units away from all walls, using the MinDist2Edges function provided for 
% % collision detection.  Use a nearest neighbour connection strategy and the 
% % CheckCollision function provided for collision checking, and find an 
% % appropriate number of connections to ensure a connection from  start to 
% % finish with high probability.
% 
% 
% % variables to store PRM components
% nS = 500; %500;  % number of samples to try for milestone creation
% milestones = [start; finish];  % each row is a point [x y] in feasible space
% edges = [];  % each row is should be an edge of the form [x1 y1 x2 y2]
% 
% disp("Time to create PRM graph")
% tic;
% % ------insert your PRM generation code here-------
% 
% % Steps:
% % 1. Generate sample
% % 2. Check for sample validity. If not valid, go back to 1.
% % 3. For each of the k closest neighbors
% % 3.1 Check if line between new point and neighbor intersects with an edge
% % 3.2 If no intersection, add the edge to the edges list
% 
% c = 1;
% r = 0;  % Keeping track of the number of rejected milestones
% a = 0;  % Keeping track of the number of accepted milestones
% k = 8;  % Use the top k neighbors for connection attempts
% while a < nS
%     % Draw a sample uniformly x in [0.5, col+0.5] y in [0.5, row+0.5]
%     x = 0.5 + rand(1) * col;
%     y = 0.5 + rand(1) * row;
%     new_point = [x y];
%     c = c+1;
% 
%     % Check for validitiy of the point using MinDist2Edges
%     d = MinDist2Edges(new_point, map);
%     if d <= 0.1
%         % Then we reject the milestone for being too close to an edge
%         r = r+1;
%         continue
%     end
%     % Then we accept the new milestone and need to check for edges
%     a = a+1;
%     milestones = [milestones; new_point];
%     % Compute a vector where each element is a squared distance to the
%     % corresponding already existing milestone
%     diff = milestones - new_point;
%     squared_distances = sum(diff.^2, 2);
%     % Then we get the top k neighbors
%     [D, I] = mink(squared_distances, k);
%     for i_index = 1:size(I)
%         m_index = I(i_index);
%         con_point = milestones(m_index, :);
%         % Now we have our proposed point and a point to attempt to connect
%         % it to. All we need to do is check if there is a wall collision
%         % for the edge that connects them.
%         [ inCollision, edge ] = CheckCollision(new_point, con_point, map);
%         if ~inCollision
%             % Then we are free to add the edge
%             % The graph is undirected so we could add two for going both
%             % ways, but we will assume we can just add one and further code
%             % will account for this
%             edges = [edges; new_point con_point];
%         end
%     end
% end
% 
% 
% 
% 
% % ------end of your PRM generation code -------
% toc;
% 
% figure(1);
% plot(milestones(:,1),milestones(:,2),'m.');
% if (~isempty(edges))
%     line(edges(:,1:2:3)', edges(:,2:2:4)','Color','magenta') % line uses [x1 x2 y1 y2]
% end
% str = sprintf('Q1 - %d X %d Maze PRM', row, col);
% title(str);
% drawnow;
% 
% print -dpng assignment1_q1.png
% 
% 
% % =================================================================
% % Question 2: Find the shortest path over the PRM graph
% % =================================================================
% %
% % Using an optimal graph search method (Dijkstra's or A*) , find the 
% % shortest path across the graph generated.  Please code your own 
% % implementation instead of using any built in functions.
% 
% disp('Time to find shortest path');
% tic;
% 
% % Variable to store shortest path
% spath = []; % shortest path, stored as a milestone row index sequence
% 
% 
% % ------insert your shortest path finding algorithm here-------
% 
% % We deal in node indices to avoid comparing float vectors
% 
% % Steps:
% % 1. Initialize variables
% % 1.1 Create the priority queue and initialize with the start milestone
% % 1.2 Initialize the dead hash map that we will be using to check whether
% %     we should expand a node
% % 1.3 Initialize the parent map that we will use to recover the path
% 
% % disp("Testing pq");
% % pq = MinPriorityQueue();
% % pq = pq.add(2, "World"); % Reassign pq to capture the updated state
% % pq = pq.add(1, "Hello ");  % Reassign pq to capture the updated state
% % pq = pq.add(3, "!");  % Reassign pq to capture the updated state
% % 
% % while ~pq.isEmpty()
% %     [priority, element, pq] = pq.pop(); % Capture the updated pq here as well
% %     disp(element);
% % end
% 
% pq = MinPriorityQueue();
% pq = pq.add(calcHeuristic(start, finish), [1, -1]);  % The start node is at index 1 with parent -1
% dead_set = false(1, size(milestones, 1));
% % So dead_set(n) being true means that node has already been visited
% parent_map = ones(1, size(milestones, 1)) * -1;
% % parent_map(n) returns the parent of milestone index n
% cost_to_come_map = ones(1, size(milestones, 1)) * -1;
% cost_to_come_map(1) = 0;
% % We initialize all cost-to-come values to -1 and then set the start to
% % have a cost-to-come of 0
% 
% while ~pq.isEmpty()
%     % Pop the lowest priority node
%     [priority, cur_node_info, pq] = pq.pop();
%     cur_node_index = cur_node_info(1);
%     cur_node_parent_index = cur_node_info(2);
%     cur_node = milestones(cur_node_index, :);
% 
%     if cur_node_parent_index >= 1
%         parent_node = milestones(cur_node_parent_index, :);
%     else
%         parent_node = [-1, -1];
%     end
%     % disp(["Processing node" cur_node_index cur_node]);
% 
%     % Then we check if this node has been processed. Since the heuristic is
%     % consistent, if the node has been processed we can skip it
%     if dead_set(cur_node_index)
%         continue
%     end
%     % Now we have processed it so we can set it to dead
%     dead_set(cur_node_index) = true;
% 
%     if cur_node_index ~= 1
%         % We can also now update the parent since we know this is the best
%         % route to the current node
%         parent_map(cur_node_index) = cur_node_parent_index;
%         % At this point we can also update the cost to come
%         cost_to_come_map(cur_node_index) = cost_to_come_map(cur_node_parent_index) + getEdgeCost(parent_node, cur_node);
%     end
% 
%     if cur_node_index == 2
%         % The finish is always the second element of the milestone list
%         break;
%     end
% 
%     % Otherwise we need to iterate over the neighbors
%     neighbors = getNeighbors(cur_node, edges);
%     for n_index = 1:size(neighbors, 1)
%         % For each neighbor, we now compute the node info and the estimated
%         % total cost and add it into the priority queue
%         neighbor = neighbors(n_index, :);
%         neighbor_index = getNodeIndex(neighbor, milestones);
% 
%         % We could check if the neighbor is dead and skip it if it is
% 
%         % Create a new node info. The current node is now the parent
%         node_info = [neighbor_index, cur_node_index];
%         % Then we need to get the priority
%         cost_to_come = cost_to_come_map(cur_node_index) + getEdgeCost(cur_node, neighbor);
%         estimated_total_cost = cost_to_come + calcHeuristic(neighbor, finish);
% 
%         pq = pq.add(estimated_total_cost, [neighbor_index, cur_node_index]);
%     end
% end
% 
% if parent_map(2) == -1
%     % Then there was no path found
%     error("No path found")
% end
% 
% % Otherwise, we can trace back through the parents to find the path
% parent_index = 2;
% while parent_index ~= -1
%     spath = [parent_index spath];
%     parent_index = parent_map(parent_index);
% end
% 
% % disp(spath)
% 
% 
% 
% 
% 
% % ------end of shortest path finding algorithm------- 
% toc;    
% 
% % plot the shortest path
% figure(1);
% for i=1:length(spath)-1
%     plot(milestones(spath(i:i+1),1),milestones(spath(i:i+1),2), 'go-', 'LineWidth',3);
% end
% str = sprintf('Q2 - %d X %d Maze Shortest Path', row, col);
% title(str);
% drawnow;
% 
% print -dpng assingment1_q2.png


% ================================================================
% Question 3: find a faster way
% ================================================================
%
% Modify your milestone generation, edge connection, collision detection 
% and/or shortest path methods to reduce runtime.  What is the largest maze 
% for which you can find a shortest path from start to goal in under 20 
% seconds on your computer? (Anything larger than 40x40 will suffice for 
% full marks)


row = 40;
col = 40;
map = maze(row,col);
start = [0.5, 1.0];
finish = [col+0.5, row];
milestones = [start; finish];  % each row is a point [x y] in feasible space
edges = [];  % each row is should be an edge of the form [x1 y1 x2 y2]

h = figure(2);clf; hold on;
plot(start(1), start(2),'go')
plot(finish(1), finish(2),'rx')
show_maze(map,row,col,h); % Draws the maze
drawnow;

fprintf("Attempting large %d X %d maze... \n", row, col);
tic;        
% ------insert your optimized algorithm here------

nS = 6000;  % We use this to be the number of valid nodes so that we can pre-allocate our adjacency list
wall_avoidance_radius = 0.1;

% An adjacency map will be efficient here as the number of neighbors is
% limited to be small
adjacency_list = cell(nS, 1);

c = 1;
r = 0;  % Keeping track of the number of rejected milestones
a = size(milestones, 1);  % Keeping track of the number of accepted milestones
k = 12;  % Use the top k neighbors for connection attempts

while a < nS
    % Draw a sample uniformly x in [0.5, col+0.5] y in [0.5, row+0.5]
    new_point = drawPoint(col, row);
    c = c+1;

    % Check for validitiy of the point using MinDist2Edges
    d = MinDist2Edges(new_point, map);
    if d <= wall_avoidance_radius
        % Then we reject the milestone for being too close to an edge
        r = r+1;
        continue
    end
    % TODO: This takes up like 20 seconds. There needs to be a better way.
    % Make an occupancy map at a low definition and then round points into
    % the map and check for collisions there by lookup. That should make
    % this go to about zero time.

    % Then we accept the new milestone and need to check for edges
    a = a+1;
    new_point_index = size(milestones, 1) + 1;

    I = getKClosest(new_point, milestones, k);
    % This does collision checking and adds the edges
    % Except that we do lazy collision checking now so it actually just
    % passes all edges through
    adjacency_list = update_adjacency(adjacency_list, I, new_point_index, milestones, map);

    % Append to the milestones afterward to avoid connecting the node to
    % itself
    milestones = update_milestones(milestones, new_point);
end

% Build the edges list from the adjacency list
% edges = build_edges(adjacency_list, milestones);

toc;

% figure(2);
% plot(milestones(:,1),milestones(:,2),'m.');
% if (~isempty(edges))
%     line(edges(:,1:2:3)', edges(:,2:2:4)','Color','magenta') % line uses [x1 x2 y1 y2]
% end
% str = sprintf('Q1 - %d X %d Maze PRM', row, col);
% title(str);
% drawnow;

spath = [];
pq = MinPriorityQueue();
pq = pq.add(calcHeuristic(start, finish), [1, -1]);  % The start node is at index 1 with parent -1
dead_set = false(1, size(milestones, 1));
% So dead_set(n) being true means that node has already been visited
parent_map = ones(1, size(milestones, 1)) * -1;
% parent_map(n) returns the parent of milestone index n
cost_to_come_map = ones(1, size(milestones, 1)) * -1;
cost_to_come_map(1) = 0;
% We initialize all cost-to-come values to -1 and then set the start to
% have a cost-to-come of 0

while ~pq.isEmpty()
    % Pop the lowest priority node
    [priority, cur_node_info, pq] = pq.pop();
    cur_node_index = cur_node_info(1);
    cur_node_parent_index = cur_node_info(2);
    cur_node = milestones(cur_node_index, :);

    if cur_node_parent_index >= 1
        parent_node = milestones(cur_node_parent_index, :);

        % Check for collision
        [ inCollision, edge ] = CheckCollision(cur_node, parent_node, map);
        if inCollision
            continue
        end
        % Add the new edge for visualization
        % We only visualize the ones we checked and found no collision
        edges = [edges; parent_node cur_node];
    else
        parent_node = [-1, -1];
    end
    % disp(["Processing node" cur_node_index cur_node]);

    % Then we check if this node has been processed. Since the heuristic is
    % consistent, if the node has been processed we can skip it
    if dead_set(cur_node_index)
        continue
    end
    % Now we have processed it so we can set it to dead
    dead_set(cur_node_index) = true;

    if cur_node_index ~= 1
        % We can also now update the parent since we know this is the best
        % route to the current node
        parent_map(cur_node_index) = cur_node_parent_index;
        % At this point we can also update the cost to come
        cost_to_come_map(cur_node_index) = cost_to_come_map(cur_node_parent_index) + getEdgeCost(parent_node, cur_node);
    end

    if cur_node_index == 2
        % The finish is always the second element of the milestone list
        disp("Found finish")
        break;
    end

    % Otherwise we need to iterate over the neighbors
    % OLD
    % neighbors = getNeighbors(cur_node, edges);
    % NEW
    neighbor_indices = getNeighborsAdjacency(cur_node_index, adjacency_list);
    for n_index = 1:size(neighbor_indices, 2)
        % For each neighbor, we now compute the node info and the estimated
        % total cost and add it into the priority queue
        % OLD
        % neighbor = neighbors(n_index, :);
        % neighbor_index = getNodeIndex(neighbor, milestones);
        % NEW
        neighbor_index = neighbor_indices(n_index);
        neighbor = milestones(neighbor_index, :);

        % We could check if the neighbor is dead and skip it if it is
        if dead_set(neighbor_index)
            continue
        end

        % Create a new node info. The current node is now the parent
        node_info = [neighbor_index, cur_node_index];
        % Then we need to get the priority
        cost_to_come = cost_to_come_map(cur_node_index) + getEdgeCost(cur_node, neighbor);
        estimated_total_cost = cost_to_come + calcHeuristic(neighbor, finish);

        pq = pq.add(estimated_total_cost, [neighbor_index, cur_node_index]);
    end
end

if parent_map(2) == -1
    % Then there was no path found
    error("No path found")
end

% Otherwise, we can trace back through the parents to find the path
parent_index = 2;
while parent_index ~= -1
    spath = [parent_index spath];
    parent_index = parent_map(parent_index);
end

% ------end of your optimized algorithm-------
toc;
dt = toc;

% Get the indices of all nodes in the dead set and not in the dead set
dead_indices = find(dead_set);
alive_indices = find(~dead_set);

figure(2); hold on;
% plot(milestones(:,1),milestones(:,2),'m.');
plot(milestones(dead_indices,1),milestones(dead_indices,2),'m.');
plot(milestones(alive_indices,1),milestones(alive_indices,2),'b.');
if (~isempty(edges))
    line(edges(:,1:2:3)', edges(:,2:2:4)','Color','magenta')
end
if (~isempty(spath))
    for i=1:length(spath)-1
        plot(milestones(spath(i:i+1),1),milestones(spath(i:i+1),2), 'go-', 'LineWidth',3);
    end
end
str = sprintf('Q3 - %d X %d Maze solved in %f seconds', row, col, dt);
title(str);

print -dpng assignment1_q3.png

function h = calcHeuristic(cur_point, goal_point)
    h = sqrt(sum((cur_point - goal_point) .^ 2));
    % h=0;
end

function neighbors = getNeighbors(point, edges)
    neighbors = [];
    for edge_index = 1:size(edges, 1)
        % Check if one of the endpoints is the point
        edge = edges(edge_index, :);
        if point(1) == edge(1) && point(2) == edge(2)
            neighbors = [neighbors; edge(3) edge(4)];
        end
        if point(1) == edge(3) && point(2) == edge(4)
            neighbors = [neighbors; edge(1) edge(2)];
        end
    end
end

function neighbor_indices = getNeighborsAdjacency(point_index, adjacency_list)
    neighbor_indices = adjacency_list{point_index};
end

function index = getNodeIndex(point, milestones)
    for node_index = 1:size(milestones, 1)
        milestone = milestones(node_index, :);
        if point(1) == milestone(1) && point(2) == milestone(2)
            index = node_index;
            return
        end
    end
    index = -1;
end

function cost = getEdgeCost(e1, e2)
    cost = sqrt(sum((e1 - e2) .^ 2));
end

function point = drawPoint(col, row)
    x = 0.5 + rand(1) * col;
    y = 0.5 + rand(1) * row;
    point = [x y];
end

function k_closest_indices = getKClosest(point, milestones, k)
    % Compute a vector where each element is a squared distance to the
    % corresponding already existing milestone
    diff = milestones - point;
    squared_distances = sum(diff.^2, 2);
    % Then we get the top k neighbors
    [D, k_closest_indices] = mink(squared_distances, k);
end

function adjacency_list = update_adjacency(adjacency_list, neighbor_indices, new_point_index, milestones, map)
    for i_index = 1:size(neighbor_indices)
        m_index = neighbor_indices(i_index);
        con_point = milestones(m_index, :);
        % Now we have our proposed point and a point to attempt to connect
        % it to. All we need to do is check if there is a wall collision
        % for the edge that connects them.
        % [ inCollision, edge ] = CheckCollision(new_point, con_point, map);
        inCollision = false;
        if ~inCollision
            % Then we are free to add the edge
            % The graph is undirected so we could add two for going both
            % ways, but we will assume we can just add one and further code
            % will account for this
            % edges = [edges; new_point con_point];
            % And we also update the adjacency list to reflect the new edge
            adjacency_list{new_point_index} = [adjacency_list{new_point_index} m_index];
            adjacency_list{m_index} = [adjacency_list{m_index} new_point_index];
        end
    end
end

function milestones = update_milestones(milestones, new_point)
    milestones = [milestones; new_point];
end

function edges = build_edges(adjacency_list, milestones)
    edges = [];
    for i = 1:size(milestones, 1)
        point = milestones(i, :);
        neighbor_indices = adjacency_list{i};
        for j = 1:size(neighbor_indices, 2)
            neighbor_index = neighbor_indices(j);
            n_point = milestones(neighbor_index, :);
            edges = [edges; point n_point];
        end
    end
end