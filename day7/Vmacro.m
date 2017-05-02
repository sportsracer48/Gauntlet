
%edit your ranges to display here.  important to not include the actual
%location of your object in this grid of points or it will give you
%infinities

[px,py]=meshgrid(-5.5:1:5.5,-5.5:1:5.5);
[xlim,ylim] = size(px);
V = zeros(xlim, ylim);

for i=1:xlim
    for j=1:ylim
%this is the equation and integral with ranges for a specific object:  you
%should be able to figure out what this is and edit appropriately to get
%what you want
dV = @(x)  1./sqrt((px(i,j)-x).^2 + py(i,j).^2).^2;
V(i,j) = integral(dV,2,4);
    end
end

for i=1:xlim
    for j=1:ylim
%this is the equation and integral with ranges for a specific object:  you
%should be able to figure out what this is and edit appropriately to get
%what you want
dV = @(y)  1./sqrt((px(i,j)).^2 + (py(i,j)-y).^2).^2;
V(i,j) = V(i,j) + integral(dV,2,4);
    end
end

for i=1:xlim
    for j=1:ylim
%this is the equation and integral with ranges for a specific object:  you
%should be able to figure out what this is and edit appropriately to get
%what you want
V(i,j) = V(i,j) + -1./sqrt((px(i,j)+1).^2+(py(i,j)+1).^2).^2;
    end
end

nSamps = 10000;
x = zeros(nSamps,1);
y = zeros(nSamps,1);
x(1) = 2;
y(1) = 2;
[Ex,Ey] = gradient(V);
for i = 2:nSamps
   x0 = x(i-1);
   y0 = y(i-1);
   gradX = interp2(px,py,Ex,x0,y0);
   gradY = interp2(px,py,Ey,x0,y0);
   x(i) = x0 - gradX*.1;
   y(i) = y0 - gradY*.1;
end

hold off
contour(px,py,V)
hold on
quiver(px,py,-Ex,-Ey)
plot(x,y,'ks');