
%edit your ranges to display here.  important to not include the actual
%location of your object in this grid of points or it will give you
%infinities

[px,py]=meshgrid(-20.5:1:20.5,-20.5:1:20.5);
[xlim,ylim] = size(px);
V = zeros(xlim, ylim);

for i=1:xlim
    for j=1:ylim
%this is the equation and integral with ranges for a specific object:  you
%should be able to figure out what this is and edit appropriately to get
%what you want
dV = @(x)  1./sqrt((px(i,j)-x).^2 + py(i,j).^2);
V(i,j) = integral(dV,-5,5);
    end
end

hold off
contour(px,py,V)
[Ex,Ey] = gradient(V);
hold on
quiver(px,py,-Ex,-Ey)