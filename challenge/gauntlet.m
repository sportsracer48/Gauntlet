function gauntlet()
    %rosinit('10.0.75.2',11311, 'NodeHost','10.0.75.1')
    pub_vel = rospublisher('/cmd_vel', 'geometry_msgs/Twist');
    sub_scan = rossubscriber('/projected_stable_scan', 'sensor_msgs/PointCloud2');
    sub_odom = rossubscriber('/odom');
    
    f = figure('CloseRequestFcn',@myCloseRequest);
    
    bucketRad = 0.1143;
    bucketLoc = [0 0];
    bucketErr = Inf;
    bucketFound = 0;
    
    boxW = 3.556;
    boxH = 2.413;
    startX = 0.3048;
    startY = 0.4572;
    startDir = 0;
    
    potentialScale = 10;
    
    running = 1;
    
    
    [pos0,rot0] = getPos();
    R0 = [cos(rot0) sin(rot0) 0;
         -sin(rot0) cos(rot0) 0;
         0          0         1];
     T0 = [1 0 -pos0(1);
           0 1 -pos0(2);
           0 0  1];
    neato2labR = R0*T0;
    lab2neatoR = inv(neato2labR);
    
    guessBucket();
    
    while(running)
       turnToGrad();
       advance(.1,.1);
       [pos,~] = getPos();
       if(~bucketFound && norm(pos-bucketLoc)<1)
           guessBucket()
       end
       if(bucketFound && norm(pos-bucketLoc)<.3)
           [s,Fs] = audioread('yay.wav');
           sound(s,Fs);
           running = 0;
           setV(0,pi/3)
           pause(6)
           setV(0,0)
           setV(0,-pi/3)
           pause(6)
           setV(0,0)
       end
    end
    
    for q = 1:10
        setV(0,0);
    end
    function advance(dist, v)
        setV(v,0);
        pause(dist/v);
        setV(0,0);
    end
    function myCloseRequest(~,~)
        delete(gcf);
        running = 0;
    end
    function res = lab2neato(v)
        res = lab2neatoR*v;
    end
    function res = randomLoc()
        minX = -startX;
        minY = -startY;
        maxX = boxW-startX;
        maxY = boxH-startY;
        x = rand()*(maxX-minX)+minX;
        y = rand()*(maxY-minY)+minY;
        res = [x y];
    end
    function guessBucket()
        bucketLoc = lab2neato([randomLoc';1])';
        bucketLoc = bucketLoc(1:2);
    end
    function res = neato2lab(v)
        res = neato2labR*v;
    end
    function turnToGrad()
        grad = getGrad();
        theta_goal = atan2(grad(2),grad(1))+pi;
        turnTo(theta_goal);
    end
    function turnTo(theta_goal)
        while(running)
            [~,theta] = getPos();
            dTheta = angdiff(theta,theta_goal);
            if(abs(dTheta)<.05)
                return;
            end
            val = dTheta;
            if(val<-1)
                val = -1;
            end
            if(val>1)
                val = 1;
            end
            setV(0,val);
        end
    end
    function res = getGrad()
        pts = readXYZ(receive(sub_scan));
        bucketPts = findBucket(pts(:,1),pts(:,2));
        
        avoid = pts(~bucketPts,:);
        goal = pts(bucketPts,:);
        
        [pos,rot] = getPos();
        x = pos(1);
        y = pos(2);
        nSamples = 25;
        sampleLength = 1;
        center = ceil(nSamples/2);
        xs = linspace(x-sampleLength/2,x+sampleLength/2,nSamples);
        ys = linspace(y-sampleLength/2,y+sampleLength/2,nSamples);
        [px, py] = meshgrid(xs,ys);
        U = zeros(nSamples);
        
        
        nPoints = size(avoid,1);
        for ptIndex = 1:nPoints
            pt = avoid(ptIndex,1:2);
            for i = 1:nSamples
                for j = 1:nSamples
                    samplePt = [px(i,j), py(i,j)];
                    U(i,j) = U(i,j) + potentialScale/norm(samplePt-pt);
                end
            end
        end
        
        nPoints = size(goal,1);
        for ptIndex = 1:nPoints
            pt = goal(ptIndex,1:2);
            for i = 1:nSamples
                for j = 1:nSamples
                    samplePt = [px(i,j), py(i,j)];
                    U(i,j) = U(i,j) + 10*potentialScale*norm(samplePt-pt);
                end
            end
        end
        

        for i = 1:nSamples
            for j = 1:nSamples
                samplePt = [px(i,j), py(i,j)];
                U(i,j) = U(i,j) + 200*potentialScale*norm(samplePt-bucketLoc);
            end
        end

        [Gx,Gy] = gradient(U);
        norms = sqrt(Gx.^2 + Gy.^2);
        Gx = Gx./norms;
        Gy = Gy./norms;
        norms = min(norms,300);
        Gx = Gx.*norms;
        Gy = Gy.*norms;
        
        res = [Gx(center,center) Gy(center,center)];
        %plotGrad(px, py, U, x, y, rot, Gx, Gy, xs, ys, goal, avoid);
        drawLab(goal, avoid, pos)
    end
    function plotGrad(px, py, U, x, y, rot, Gx, Gy, xs, ys, goal, avoid)
        clf
        hold on
        contour(px, py, U);
        plot(x,y,'bo')
        quiver(x,y,cos(rot),sin(rot))
        norms = sqrt(Gx.^2 + Gy.^2);
        quiver(xs,ys,Gx./norms,Gy./norms);
        plot(goal(:,1),goal(:,2),'gx');
        plot(avoid(:,1),avoid(:,2),'rx');
        axis equal
        hold off
        drawnow
    end
    function drawLab(goal, avoid, pos)
        goal(:,3) = ones(size(goal,1),1);
        avoid(:,3) = ones(size(avoid,1),1);
        
        goal = neato2lab(goal')';
        avoid = neato2lab(avoid')';
        pos = neato2lab([pos'; 1]);
        bucketPos = neato2lab([bucketLoc'; 1]);
        
        clf
        hold on
        plot(goal(:,1),goal(:,2),'gx');
        plot(avoid(:,1),avoid(:,2),'rx');
        plot(pos(1),pos(2),'bo');
        plot(bucketPos(1),bucketPos(2),'go');
        axis equal
        hold off
        drawnow
    end
    function [pos,theta] = getPos()
        odom_pose = receive(sub_odom);
        trans = odom_pose.Pose.Pose.Position;
        quat = odom_pose.Pose.Pose.Orientation;
        rot = quat2eul([quat.W quat.X quat.Y quat.Z]);
        pos = [trans.X, trans.Y];
        theta = rot(1);
    end
    function setV(vLin, vAng)
        msg = rosmessage(pub_vel);
        msg.Linear.X = vLin;
        msg.Angular.Z = vAng;
        send(pub_vel,msg);
    end
    function res = dist2line(x, y, start, vect)
        M = [x y];
        M = M - start';
        normal = [-vect(2) ; vect(1)];
        normal = normal/norm(normal);
        res = abs(M * normal);
    end
    function res = distOnLine(x, y, start, vect)
        M = [x y];
        M = M - start';
        projVect = vect/norm(vect);
        res = M * projVect;
    end
    function [bestStart,bestVect,ptsOnLine] = findLine(x, y, epsilon, nTries)
        nPoints = length(x);

        bestStart = [0; 0];
        bestVect = [0; 0];
        bestScore = -Inf;
        ptsOnLine = [];

        for succ = 1:nTries
            i=0;
            j=0;
            while(i==j)
                i = randi(nPoints);
                j = randi(nPoints);
            end
            a = [x(i);y(i)];
            b = [x(j);y(j)];
            vect = b-a;
            dist = dist2line(x,y,a,vect);
            score = sum(dist < epsilon);
            if(score > bestScore)
                bestScore = score;
                bestStart = a;
                bestVect = vect;
                ptsOnLine = dist < epsilon;
            end
        end
        ptsOnLine = removeGaps(x, y, ptsOnLine, bestStart, bestVect, .3);
        [bestStart, bestVect] = linRegPCA(x(ptsOnLine), y(ptsOnLine));
    end
    function pts = removeGaps(x, y, ptsOnLine, start, vect, epsilon)
        xsOnLine = x(ptsOnLine);
        ysOnLine = y(ptsOnLine); 
        ts = distOnLine(xsOnLine, ysOnLine, start, vect/norm(vect));
        [tsSorted,indicies] = sort(ts);
        diffs = diff(tsSorted);
        meanDiff = mean(diffs);
        breaks = [0; find(diffs>epsilon); length(ts)];
        lengths = diff(breaks);
        [~,maxSegment] = max(lengths);
        indiciesOnLine = breaks(maxSegment)+1:breaks(maxSegment+1);
        localIndicies = indicies(indiciesOnLine);

        L = [xsOnLine(localIndicies) ysOnLine(localIndicies)];
        G = [x y];

        pts = ismember(G, L, 'rows');
    end
    function [cent,r,err] = getCircle(x, y)
        nPoints = length(x);
        A = [x y ones(nPoints,1)];
        b = -x.^2-y.^2;
        w = A\b;
        cent = [-w(1)/2 -w(2)/2];
        r = sqrt(cent(1)^2 + cent(2)^2 -w(3));

        M = [x y] - cent;
        r_actual = sqrt(diag(M*M'));
        err = mean((r_actual-r).^2);
    end
    function res = onCircle(cent, r, x, y, epsilon)
        M = [x y] - cent;
        r_actual = sqrt(diag(M*M'));
        res = abs(r_actual-r)<epsilon;
    end
    function bucketPts = findBucket(x, y)
        XY = [x y];
        %bucketPts = false(length(x),1);
        i=1;
        while(length(x)>2)
            [~,~,ptsOnLine] = findLine(x, y, .01, 100);
            [cent, r, err] = getCircle(x(ptsOnLine),y(ptsOnLine));
            if(abs(r-bucketRad)<.05 && sum(ptsOnLine)>5 && err<1e-6)
                if(err<bucketErr)
                    if(~bucketFound)
                        [s,Fs] = audioread('ahh.wav');
                        sound(s,Fs);
                    end
                    bucketFound = 1;
                    bucketErr = err
                    bucketLoc = cent
                end
            end
            x = x(~ptsOnLine);
            y = y(~ptsOnLine);  
            i=i+1;
        end
        x = XY(:,1);
        y = XY(:,2);
        bucketPts = onCircle(bucketLoc, bucketRad, x, y, .05);
    end
    function [mu,v] = linRegPCA(x,y)
        XY = [x y];
        A = XY;
        mu = mean(A);
        A = A - mu;
        mu = mu';
        [~, ~, V] = svd(A,'econ');
        v = V(:,1);
        ts = distOnLine(x, y, mu, v);
        [tmin,minIndex] = min(ts);
        tmax = max(ts);
        mu = XY(minIndex,:)';
        len = tmax-tmin;
        v = v * len;
    end
end

