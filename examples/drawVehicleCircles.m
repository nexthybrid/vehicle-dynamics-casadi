function p = drawVehicleCircles(X_c,Y_c,psi,d,R,varargin)
%drawVehicleCircles Draw vehicle Ziegler 3-circle shape
%   p = drawVehicleCircles(X_c,Y_c,psi,d,R,colors)

%   Inputs:
%   X_c, Y_c: 1xn double , vehicle CG X,Y coordinates [m]
%   psi: 1xn double , yaw angle [rad]
%   d: double, neiguboring circle center distance [m]
%   R: double, vehicle width [m]
%   colors: 3x3 double[0-1], vehicle 3-circle colors (front,mid,rear)
%   Output:
%   p: 1x3 handles, the line object handles to the three circles.

%   The math:
%   For the three circle centers [x_f,y_f], [x_m,y_m], [x_r,y_r]
%   x_f = X_c + d*cos(psi);
%   y_f = Y_c + d*sin(psi);
%   x_m = X_c;
%   y_m = Y_c;
%   x_r = X_c - d*cos(psi);
%   y_r = Y_c - d*sin(psi);

Defaults = {[0 0 1; 0 1 0; 1 0 0]};
Defaults(1:nargin-5) = varargin;
colors = Defaults{1}; % edge colors (front, mid, rear)

x_f = X_c + d*cos(psi);
y_f = Y_c + d*sin(psi);
x_m = X_c;
y_m = Y_c;
x_r = X_c - d*cos(psi);
y_r = Y_c - d*sin(psi);

for j = 1:length(X_c)
    p(1) = circle(x_f(j),y_f(j),R,colors(1,:));
    p(2) = circle(x_m(j),y_m(j),R,colors(2,:));
    p(3) = circle(x_r(j),y_r(j),R,colors(3,:));
end
axis equal;
end

function hCirc = circle(x,y,r,varargin)
%circle Draw a circle
%   hCirc = circle(x,y,r,colorr) draws a circle centered at [x,y] with a
%   radius of r, with the color of colorr
%   0.01 is the angle step, bigger values will draw the circle faster but
%   you might notice imperfections (not very smooth).
    Defaults = {[0 0 0]};
    Defaults(1:nargin-3) = varargin;
    colorr = Defaults{1};
    ang=0:0.01:2*pi; 
    xp=r*cos(ang);
    yp=r*sin(ang);
    hCirc = plot(x+xp,y+yp,'Color',colorr);
end