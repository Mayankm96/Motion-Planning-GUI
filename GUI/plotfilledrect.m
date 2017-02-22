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