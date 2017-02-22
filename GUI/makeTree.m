function makeTree(Obstacles,T1,T2)
%Runs the iterations to form the trees
fl=0;
% Number of vertices
K=10000;
if nargin<3
    c='b';
    i=2;
    while i<K
        i=T1.buildTree(Obstacles,i,c,1);
        if T1.hasReachedGoal
            T1.showPath(c);
            fl=1;
        end
        if fl==1
            break;
        end
        i=i+1;
    end
end

if nargin==3
    c1='g';
    c2='r';
    i=2;
    j=2;
    while i<K
        i=T1.buildTree(Obstacles,i,c1,1);
        j=T2.buildTree(Obstacles,j,c2,1);
        T2=T1.connect(T2);
        if T1.hasReachedGoal
            T1.showPath(c1);
            fl=1;
        end
        if T2.hasReachedGoal
            T2.showPath(c2);
            fl=1;
        end
        if fl==1
            break;
        end
        i=i+1;
        j=j+1;
    end
end