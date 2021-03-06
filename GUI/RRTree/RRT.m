% The MIT License
% 
% Copyright (c) 2017 Mayank Mittal
% 
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in
% all copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
% THE SOFTWARE.

classdef RRT < handle 
    %Class implementation of the Rapidly Exploring Random Trees
    %   Made by Mayank Mittal
    
    properties (SetAccess=private)
        nodes={[]};
        edges={[]};
        start=[0;0];
        goal=[500;500];
        radius=10;
        Obstacles=[];
    end
    
    methods
        function Tree=RRT(S,G,r,O)
            Tree.start=S;
            Tree.goal=G;
            Tree.nodes{1}=S;
            Tree.radius=r;
            Tree.Obstacles=O;
        end
        function makeTree(T1,fc1,flag,T2,fc2)
        %Runs the iterations to form the trees
        %   flag is boolean variable. If flag=1 means only final path has to be
        %   shown and not the growing tree
        fl=0;
        if nargin==3
            i=2;
            while i<10000
                i=T1.buildTree(i,fc1,flag);
                if T1.hasReachedGoal
                    T1.showPath(fc1);
                    fl=1;
                end
                if fl==1
                    break;
                end
                i=i+1;
            end
        elseif  nargin==5
            i=2;
            j=2;
            while i<10000
                i=T1.buildTree(i,fc1,flag);
                j=T2.buildTree(j,fc2,flag);
                T2=T1.connect(T2);
                if T1.hasReachedGoal
                    T1.showPath(fc1);
                    fl=1;
                end
                if T2.hasReachedGoal
                    T2.showPath(fc2);
                    fl=1;
                end
                if fl==1
                    break;
                end
                i=i+1;
                j=j+1;
            end
        else
            error('Insufficient arguments to grow the Tree');
        end
        end
    end
    
    methods (Access=private) 
        function i=buildTree(Tree,i,fc,flag)
        %Builds a tree from the defined start point to goal point
        %   Obstacles is an array of obstacles defined
        %   pf is boolean Flag which is 0 to see the tree expanding
            x_rand=Tree.random_state(1000);
            x_near=Tree.nearest_neighbour(x_rand);
            if Tree.isPathClear(x_rand, x_near)
                [x_new,u]=Tree.new_state(x_rand,x_near);
                if ~Tree.isInGraph(x_new)
                    Tree.nodes{i}=x_new;
                    Tree.edges{i-1}={x_near,u,x_new};
                    if flag==0
                        plot([x_new(1); x_near(1)],[x_new(2); x_near(2)],fc)
                        hold on
                        pause(0.005);
                    end
                end
            else
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
            u=[norm(Tree.goal-x), fulltan(Tree.goal(2)-x(2),Tree.goal(1)-x(1))];
            Tree.edges{i}={x,u,Tree.goal};
            flag=1;
        else
            flag=0;
        end
        end
        function Tree2= connect(Tree1,Tree2)
        %Connects the branches of two trees if they are nearby
        %   Returns 1 if the trees got connected
            for i=1: length(Tree1.nodes)
                x1=Tree1.nodes{i};
                for j=1:length(Tree2.nodes)
                    x2=Tree2.nodes{j};
                    if norm(x1-x2)<Tree1.radius 
                        Tree1.goal=x2;
                        Tree2.goal=x1;
                        % Add connecting branch on Tree2
                        k=length(Tree2.nodes);
                        Tree2.nodes{k+1}=Tree2.goal;
                        u=[1, fulltan(x2(2)-x1(2),x2(1)-x1(1))];
                        Tree2.edges{k}={x2,u,Tree2.goal};
                        % Add connecting branch on Tree1
                        k=length(Tree1.nodes);
                        Tree1.nodes{k+1}=Tree1.goal;
                        u=[1, fulltan(x1(2)-x2(2),x1(1)-x2(1))];
                        Tree1.edges{k}={x1,u,Tree1.goal};
                    end
                end
            end
        end
        function showPath(Tree,fc)
        %Displays path from given state back to the start
        %   S: Start point
        %   edges: list of all edges of graph {x_i, u, x_i+1}
        i=length(Tree.edges);
        x=Tree.goal;
        while x~=Tree.start
            if Tree.edges{i}{3}==x
                xs=Tree.edges{i}{1};
                plot([xs(1); x(1)],[xs(2); x(2)],fc,'LineWidth',3);
                x=xs;
            end
            i=i-1;
        end
        end
        function flag= isPathClear(Tree, x_rand, x_near)
        %Checks if the path is clear between tentaive new node and nearest node in tree
        %   Returns 1 if given path is free of obstacles
            flag=1;
            x=[linspace(x_near(1),x_rand(1),100);linspace(x_near(2),x_rand(2),100)];
            i=2;
            while i<=length(x)
                if Tree.isColliding(x(:,i),Tree.Obstacles)
                    flag=0;
                    break;
                end
                i=i+1;
            end
        end
        function [x_new,u]= new_state(Tree,x_rand,x_near)
        %Used to generate a new state from a given state xi
        %   Since we are doing holonmic planning \dot(x)=f(x,u)=u 
        %   This allows new state to be generated in any direction
        %   Action u can be best understood as a velocity vector
        %   u=[magnitude; angle]
            x_new=x_rand;
            u=[norm(x_new-x_near), fulltan(x_new(2)-x_near(2),x_new(1)-x_near(1))];
            if u(1)>Tree.radius
                u(1)=Tree.radius;
            end
            x_new=x_near+[u(1)*cos(u(2));u(1)*sin(u(2))];
        end
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