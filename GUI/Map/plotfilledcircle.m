function ph = plotfilledcircle(center,r,fc)
%Function to draw a circle
%   Plots a filled circle with radius 'r', center 'center' and filled color 'fc' 

theta = linspace(0,2*pi); 
x = r*cos(theta) + center(1);
y = r*sin(theta) + center(2);
ph = fill(x, y, fc);
ph=struct('Vertices',ph.Vertices,'FaceColor',ph.FaceColor);
end