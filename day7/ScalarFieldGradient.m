figure
%Change these ranges and point spacings to make nice figures
[X,Y] = meshgrid([1:0.1:10],[1:0.1:10]);
[X1,Y1] = meshgrid([1:1:10],[1:1:10]);

%Define your function below.  Don't forget to specify element by element
%operations.
syms x y
%f = 12-(x-2).^2 -(y-4).^2;
f = sin(x) + cos(y);
%f = 1/norm([x y]);
%f = cos(x)*sin(y);

%This takes the gradient of your function
g = gradient(f,[x,y])

%These lines of code make nice plots.  Look at documentation to adjust
%plotting parameters as you like.
hold off
contour(X,Y,subs(f,[x,y],{X,Y}))

G1 = subs(g(1),[x,y],{X1,Y1});
G2 = subs(g(2),[x,y],{X1,Y1});
hold on
quiver(X1,Y1,G1,G2)