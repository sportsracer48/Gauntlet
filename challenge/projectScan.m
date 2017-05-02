tf = rostf;
% wait for a bit so we can build up a few tf frames
pause(2);
sub = rossubscriber('/stable_scan');
subUnstabilized = rossubscriber('/scan');
subPointCloud = rossubscriber('/projected_stable_scan');
f = figure;
while 1
    scan = receive(sub);
    scanUnstabilized = receive(subUnstabilized);
    pointCloud = receive(subPointCloud);
    points = readXYZ(pointCloud);
    [R, T] = getScanTransform(tf, scan.Header.Stamp);
    [Ru, Tu] = getScanTransform(tf, scanUnstabilized.Header.Stamp);
    if isempty(Ru) | isempty(R)
        disp('skipping scan');
        continue
    end
    rs = scan.Ranges(1:end-1);
    theta = [0:359]';
    rclean = rs(rs ~= 0);
    thetaclean = theta(rs ~= 0);
    data = [(rclean.*cosd(thetaclean))';...
            (rclean.*sind(thetaclean))';...
            ones(size(rclean'))];
    dataTransformed = T*R*data;
    dataTransformedU = Tu*Ru*data;
    set(0, 'CurrentFigure', f);
    subplot(3, 1, 1);
    scatter(dataTransformed(1,:), dataTransformed(2,:), '.');
    title('With Scan Time Optimization');
    axis equal;
    hold on;
    subplot(3, 1, 2);
    scatter(dataTransformedU(1,:), dataTransformedU(2,:), '.');
    title('Without Scan Time Optimization');
    axis equal;
    hold on;
    subplot(3, 1, 3);
    title('With All Optimizations');
    scatter(points(:,1), points(:,2), '.');
    axis equal;
    hold on;
end