function calibration()
    %rosinit('10.0.75.2',11311, 'NodeHost','10.0.75.1')
    d = 0.24*1.064;
    pub_vel = rospublisher('/raw_vel');
    sub_accel = rossubscriber('/accel');
    
    function setWheelV(vl,vr)
        msg = rosmessage(pub_vel);
        msg.Data = [vl vr];
        %disp(msg.Data)
        send(pub_vel,msg); 
    end
    function [vl,vr] = calculateV(vLinear, vAngular)
        vl = vLinear - (vAngular*d)/2;
        vr = vLinear + (vAngular*d)/2;
    end
    function setV(vLin, vAng)
        [vl,vr] = calculateV(vLin,vAng);
        setWheelV(vl, vr);
    end
    
    msg = receive(sub_accel);
    disp(msg.Data)
    
    setV(0,pi/3)
    pause(6)
    setV(0,0)
end