function workspace(size)
%Draws a rectangular workspace and workspace obstacles as defined 
%   draw workspace rectangle of size 'size'
%   S: Start point in 2D plane
%   G: Goal point in 2D plane
rectangle('Position',[0 0 size size],'EdgeColor','k','LineWidth',3);
axis([0 size 0 size]); 
hold on;
axis square;
end