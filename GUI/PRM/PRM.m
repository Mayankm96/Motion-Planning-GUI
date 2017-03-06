classdef PRM < handle 
    %A class implementation of the Probabilistic RoadMap with A* search
    %   Made by Mayank Mittal
    %   Note: Edges are being stored in the form of adjacency list
    
    properties (SetAccess=private)
        nodes={[]};
        edges={[]};
        start=[0;0];
        goal=[500;500];
        size=1000;
        Obstacles;
    end
    
    methods
        function RoadMap=PRM(S,G,size,Obstacles)
            RoadMap.start=S';
            RoadMap.goal=G';
            RoadMap.nodes=[RoadMap.start; RoadMap.goal];
            RoadMap.size=size;
            RoadMap.Obstacles=Obstacles;
        end
        function makePRM(RoadMap,fc,flag)
        %Runs the roadmap algorithm to find shortest path to goal if any
        %   flag is boolean variable, 
        %   flag=1 means only final path has to be  shown 
            RoadMap.samplePRM(flag);
            RoadMap.joinPRM(fc,flag);
            RoadMap.findPath(fc);
        end
    end
    
    methods (Access=private)
        function samplePRM(RoadMap,flag)
        %Samples the points for the RoadMap from the defined start point 
        %   Obstacles is an array of obstacles defined
        %   flag is bool flag which is 1 if only final path needs to be shown
        
        %RoadMap.nodes has first two nodes as query points
        i=3;
        while i<(RoadMap.size+1)
            x_new=RoadMap.random_state(1000);
            if( ~RoadMap.isInGraph(x_new) && ~RoadMap.isColliding(x_new,RoadMap.Obstacles))
                RoadMap.nodes=[RoadMap.nodes; x_new];                      
                if flag==0
                    scatter(x_new(1),x_new(2),'b','filled');
                    hold on
                    pause(0.005);
                end
            else
                i=i-1;
            end
            i=i+1;
        end
        end
        function joinPRM(RoadMap,fc,flag)
        %Samples the points for the RoadMap from the defined start point 
        %   Obstacles is an array of obstacles defined
        %   flag is bool flag which is 1 if only final path needs to be shown
        RoadMap.edges=cell(RoadMap.size);
        for i=1:RoadMap.size
            x1=RoadMap.nodes(i,:);
            for j=(i+1):RoadMap.size
                x2=RoadMap.nodes(j,:);
                if RoadMap.isPath(x1,x2)
                    RoadMap.edges{i}=[RoadMap.edges{i};j];
                    RoadMap.edges{j}=[RoadMap.edges{j};i];
                    if flag==0
                        plot([x1(1), x2(1)],[x1(2), x2(2)],fc);
                        hold on
                        pause(0.005);
                    end
                end
            end
        end
        end
        function findPath(RoadMap,fc)
        %Implements A* Algorithm to find the shortest path to goal
        %   fc is the color of the path

        %Vertex= [index of node in RoadMap.nodes, total cost (fScore), historic cost (gScore), parent index] 
        openSet=[1, RoadMap.heuristic(RoadMap.nodes(1,:)), 0, -1]; 
        closedSet=[];              
        pathFound=0;
        
        while size(openSet,1)>0
             [~, index]=min(openSet(:,2));                                  % node having least total cost   
             current=openSet(index,:);                                      % smallest cost element to process
             if current(1)==2                                               % if current is the goal 
                 pathFound=1;
                 break;
             end
             openSet=[openSet(1:index-1,:); openSet(index+1:end,:)];        % remove current from openSet
             closedSet=[closedSet; current];                                % update closed lists
             
             for in=1:length(RoadMap.edges{current(1)})                     %iterate through all edges from the node
                 nb_index=RoadMap.edges{current(1)}(in);
                 if isempty(closedSet) || ~any(closedSet(:,1)==nb_index)    % checks if neighbour not present in closedSet
                     historicCost=RoadMap.historic(RoadMap.nodes(current(1),:),RoadMap.nodes(nb_index,:));
                     tentative_historicCost=current(3)+historicCost;

                     if any(openSet(:,1)==nb_index)
                        index=find(openSet(:,1)==nb_index);
                        if openSet(index,3)<tentative_historicCost
                            continue;
                        end
                     else
                         totalCost=tentative_historicCost+RoadMap.heuristic(RoadMap.nodes(nb_index,:));
                         openSet=[openSet; nb_index, totalCost, tentative_historicCost, current(1)];
                     end
                 end           
             end
        end
        
        if ~pathFound
            error('No Path Found due to Insufficient Sampling')
        else
            path=RoadMap.nodes(current(1),:);                             
            prev=current(4);
            while prev>0
                path=[RoadMap.nodes(closedSet(prev,1),:); path];
                plot(path(:,1),path(:,2),fc,'LineWidth',3);
                prev=closedSet(prev,4);
            end
            plot(path(:,1),path(:,2),fc,'LineWidth',3);
        end
        end
        function flag = isInGraph(RoadMap,x)
        %Checks whether the creared node already present in graph
        %   Returns 0 if created node not in graph
        flag=0;
        for i=1:size(RoadMap.nodes,1)
            if x==RoadMap.nodes(i,:)
                flag=1;
            end
        end
        end
        function flag= isPath(RoadMap,x1,x2)
        %Checks if two nodes x1 and x2 can be connected without obstruction
        %   Returns 1 if valid connection feasible
        flag=1;
        x=[linspace(x1(1),x2(1),100);linspace(x1(2),x2(2),100)];
        i=2;
        while i<=length(x)
            if RoadMap.isColliding(x(:,i),RoadMap.Obstacles)
                flag=0;
                break;
            end
            i=i+1;
        end
        end
        function h=heuristic(RoadMap,x)
        % Computes the heuristic function for A* algorithm
        %   Computes Euclidean distance to the goal as heauristic function
        h = norm(x-RoadMap.goal);
        end
    end
    
    methods (Static)
        function flag = isColliding(x,Obstacles)
        %Checks for collision between pose x and workspace obstacles
        %   Returns 1 if given pose x leads to collision
        flag=0;
        for i=1:length(Obstacles)
            if ~isequal(inpolygon(x(1),x(2),Obstacles{i}.Vertices(:,1),Obstacles{i}.Vertices(:,2)),zeros(1,length(x(1))));
                flag=1;
            end
        end
        end
        function x = random_state(s)
        %Returns a random state in the free workspace
        %   s: Size of workspace
        %   O: array of obstacles
            x=rand(1,2)*s;
        end
        function h=historic(x1,x2)
        % Computes the historic function for A* algorithm
        %   Uses Euclidean norm
        h = norm(x1-x2);
        end
    end
end