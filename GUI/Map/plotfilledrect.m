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

function rect=plotfilledrect(center,l,b,phi,fc)
%Function to draw a rectangle
%   Plots a filled rectangle with center 'center' and filled color 'fc'
%   Rectangle has length 'l', breadth 'b' and angle orientation 'phi' in
%   degrees
phi=phi*pi/180;
X=[l/2,l/2,-l/2,-l/2];
Y=[b/2,-b/2,-b/2,+b/2];
P=[cos(phi), -sin(phi); sin(phi), cos(phi)]*[X;Y];
P=repmat(center',1,4)+P;
rect=fill(P(1,:), P(2,:), fc);
rect=struct('Vertices',rect.Vertices,'FaceColor',rect.FaceColor);
end