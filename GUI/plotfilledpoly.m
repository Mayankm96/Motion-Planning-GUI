function poly= plotfilledpoly(center,n,r,fc)
%Function to draw a regular polygon
%   Plots a filled regular polygon with center 'center' and filled color 'fc'
%   The polygon has sides 'n', and is circumscribed in a circle of radius 'r'
t = (1/(2*n):1/n:1)'*2*pi;
X = center(1)+r*cos(t);
Y = center(2)+r*sin(t);
poly=fill(X,Y,fc);
poly=struct('Vertices',poly.Vertices,'FaceColor',poly.FaceColor);
end