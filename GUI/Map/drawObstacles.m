function O = drawObstacles(i)
%Creates the obstacles as defined in this function for the workspace drawn
%   Returns an array of obstacles created

switch i
    case 2
        rect1=plotfilledrect([400,600],800,60,90,'k');
        hold on;
        rect2=plotfilledrect([200,400],800,60,90,'k');
        rect3=plotfilledrect([600,400],800,60,90,'k');
        rect4=plotfilledrect([800,600],800,60,90,'k');
        O=[rect1,rect2,rect3,rect4];
    case 3
        rect=plotfilledrect([300,750],400,100,0,'k');
        tri=plotfilledpoly([800,550],3, 125,'k');
        cir=plotfilledcircle([300,250], 110,'k');
        O=[rect,tri,cir];
    otherwise
        O=[];
end
end

