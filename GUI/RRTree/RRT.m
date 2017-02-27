classdef RRT < handle 
    %A class implementation of the Rapidly Exploring Random Trees
    %   Made by Mayank Mittal
    
    properties (SetAccess=private)
        nodes={[]};
        edges={[]};
        start=[0;0];
        goal=[500;500];
        radius=10;
        dt=10;
    end
    
    methods
        function Tree=RRT(S,G,r,dt)
            Tree.start=S;
            Tree.goal=G;
            Tree.nodes{1}=S;
            Tree.radius=r;
            Tree.dt=dt;
        end
        function i=buildTree(Tree,Obstacles,i,fc,pf)
        %Builds a tree from the defined start point to goal point
        %   Obstacles is an array of obstacles defined
        %   pf is Plot Flag which is 1 to see the growing tree
            x_rand=Tree.random_state(1000);
            x_near=Tree.nearest_neighbour(x_rand);
            [x_new,u,Tree.dt]=Tree.new_state(x_rand,x_near);
            if( ~Tree.isInGraph(x_new) && ~Tree.isColliding(x_new,Obstacles))
                Tree.nodes{i}=x_new;
                Tree.edges{i-1}={x_near,u,Tree.dt,x_new};
                if pf==1
                    plot([x_new(1); x_near(1)],[x_new(2); x_near(2)],fc)
                    hold on
                    pause(0.005);
                end
            else
                i=i-1;
            end
        end
        function showPath(Tree,fc)
        %Displays path from given state back to the start
        %   S: Start point
        %   edges: list of all edges of graph {x_i, u, dt, x_i+1}
        i=length(Tree.edges);
        x=Tree.goal;
        while x~=Tree.start
            if Tree.edges{i}{4}==x
                xs=Tree.edges{i}{1};
                plot([xs(1); x(1)],[xs(2); x(2)],fc,'LineWidth',3);
                x=xs;
            end
            i=i-1;
        end
        end
        function flag = hasReachedGoal(Tree)
        %Checks whether the state reached is within a bounded region around goal
        %   Returns 1 if it has reached goal
        i=length(Tree.nodes);
        x=Tree.nodes{i};
        if norm(x-Tree.goal)<=Tree.radius
            Tree.nodes{i+1}=Tree.goal;
            u=[1, fulltan(Tree.goal(2)-x(2),Tree.goal(1)-x(1))];
            Tree.edges{i}={x,u,Tree.dt,Tree.goal};
            flag=1;
        else
            flag=0;
        end
        end
        %function Tree2=copy2Tree(Tree1,Tree2,i)
        %Appends nodes in Tree1 from node i to start into Tree2
%             l=length(Tree2.nodes)+1;
%             x=Tree1.nodes{i};
%             while x~=Tree1.start
%                 Tree2.nodes{l}=x;
%                 if Tree1.edges{i}{4}==x
%                     xs=Tree1.edges{i}{1};
%                     u=[1, fulltan(xs(2)-x(2),xs(1)-x(1))];
%                     Tree2.edges{l}={x,u,Tree2.dt,xs};
%                     x=xs;
%                 end
%                 l=l+1;
%                 i=i-1;
%             end
%        end
        function Tree2= connect(Tree1,Tree2)
        %Connects the branches of two trees if they are nearby
        %   Returns 1 if the trees got connected
            for i=1: length(Tree1.nodes)
                x1=Tree1.nodes{i};
                for j=1:length(Tree2.nodes)
                    x2=Tree2.nodes{j};
                    if norm(x1-x2)<Tree1.radius 
                        %Tree2=Tree1.copy2Tree(Tree2,i);
                        %Tree1=Tree2.copy2Tree(Tree1,j);
                        Tree1.goal=x2;
                        Tree2.goal=x1;
                        % Add connecting branch on Tree2
                        k=length(Tree2.nodes);
                        Tree2.nodes{k+1}=Tree2.goal;
                        u=[1, fulltan(x2(2)-x1(2),x2(1)-x1(1))];
                        Tree2.edges{k}={x2,u,Tree2.dt,Tree2.goal};
                        % Add connecting branch on Tree1
                        k=length(Tree1.nodes);
                        Tree1.nodes{k+1}=Tree1.goal;
                        u=[1, fulltan(x1(2)-x2(2),x1(1)-x2(1))];
                        Tree1.edges{k}={x1,u,Tree1.dt,Tree1.goal};
                        
                        disp('Connected');
                    end
                end
            end
        end
    end
    
    methods (Access=private)
        function flag = isInGraph(Tree,x_new)
        %Checks whether the creared node already present in graph
        %   Returns 0 if created node not in graph
        flag=0;
        for i=1:length(Tree.nodes)
            if isequal(x_new,Tree.nodes{i})
                flag=1;
            end
        end
        end
        function x_near = nearest_neighbour(Tree,x)
        %Finds the nearest node in tree to given state x
        %   Returns the index of the nearest neighbour to x
        d=zeros(1,length(Tree.nodes));
        for i=1:length(Tree.nodes)
            d(i)=norm(Tree.nodes{i}-x);
        end
        [~,in]=min(d);
        x_near=Tree.nodes{in};
        end
    end
    
    methods (Static)
        function [x_new,u,dt]= new_state(x_rand,x_near)
        %Used to generate a new state from a given state xi
        %   Since we are doing holonmic planning \dot(x)=f(x,u)=u with norm(u)<1
        %   This allows new state to be generated in any direction interval dt
        %   Action u can be best understood as a velocity vector
        %   u=[magnitude; angle]
            u=[1, fulltan((x_rand(2)-x_near(2)),(x_rand(1)-x_near(1)))];

            %Assuming finite step time size
            dt=10;
            u_ac=[u(1)*cos(u(2)); u(2)*sin(u(2))];
            x_new=x_near+u_ac*dt;
        end
        function flag = isColliding(x,Obstacles)
        %Checks for collision between pose x and workspace obstacles
        %   Returns 1 if given configuration leads to collision
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
            x=rand(2,1)*s;
        end
    end
end