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
        O={rect1,rect2,rect3,rect4};
    case 3
        rect=plotfilledrect([300,750],400,100,0,'k');
        tri=plotfilledpoly([800,550],3, 125,'k');
        cir=plotfilledcircle([300,250], 110,'k');
        O={rect,tri,cir};
    otherwise
        O={};
end
end

