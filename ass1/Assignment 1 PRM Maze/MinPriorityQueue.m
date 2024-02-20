% classdef MinPriorityQueue
%     properties (Access = private)
%         pq % The priority queue stored as a cell array
%     end
% 
%     methods
%         function obj = MinPriorityQueue()
%             % Constructor to create a new min priority queue
%             obj.pq = {}; % Initialize as an empty cell array
%         end
% 
%         function isEmpty = isEmpty(obj)
%             % Checks if the priority queue is empty
%             isEmpty = isempty(obj.pq);
%         end
% 
%         function obj = add(obj, priority, element)
%             % Adds a member to the priority queue
%             obj.pq = [obj.pq; {priority, element}]; % Add as a new row in the cell array
%         end
% 
%         function [priority, element, obj] = pop(obj)
%             % Pops and returns the element with the lowest priority in the queue
%             if isempty(obj.pq)
%                 error('Priority queue is empty');
%             end
%             priorities = cell2mat(obj.pq(:, 1));
%             [priority, I] = min(priorities);
%             element = obj.pq{I, 2};
%             obj.pq(I, :) = []; % Remove the element from the queue
%         end
%     end
% end

classdef MinPriorityQueue
    properties (Access = private)
        priorities % An array to store priorities
        elements % A cell array to store elements
    end
    
    methods
        function obj = MinPriorityQueue()
            % Constructor to create a new min priority queue
            obj.priorities = []; % Initialize as an empty array
            obj.elements = {}; % Initialize as an empty cell array
        end
        
        function isEmpty = isEmpty(obj)
            % Checks if the priority queue is empty
            isEmpty = isempty(obj.priorities);
        end
        
        function obj = add(obj, priority, element)
            % Adds a member to the priority queue
            obj.priorities = [obj.priorities; priority]; % Add priority to the priorities array
            obj.elements = [obj.elements; {element}]; % Add element to the elements cell array
        end
        
        function [priority, element, obj] = pop(obj)
            % Pops and returns the element with the lowest priority in the queue
            if isempty(obj.priorities)
                error('Priority queue is empty');
            end
            [priority, I] = min(obj.priorities); % Find the min priority and its index
            element = obj.elements{I}; % Retrieve the element with the lowest priority
            obj.priorities(I) = []; % Remove the priority from the array
            obj.elements(I) = []; % Remove the element from the cell array
        end
    end
end