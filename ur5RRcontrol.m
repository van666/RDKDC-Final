function [finalerr] =ur5RRcontrol(q, gdesired, Tstep)
    ur5 = ur5_interface();
%     Tstep = 0.1;
    K = 0.3;
  
    %     q = [pi/4;0;pi/3;pi/4;0;pi/4];
    
    %     gd_inv=inv(gdesired);
    gd_inv = inv(gdesired);
    
    desiredFrame = tf_frame('base_link','desiredFrame',eye(4));
    desiredFrame.move_frame('base_link',gdesired);
    
    while 1
        gst = ur5FwdKin(q);
        g_error = gd_inv * gst;
        s = getXi(g_error);
    
        q=q-K*Tstep*inv(ur5BodyJacobian(q))*s;
    
        % check singulariries; if failure, return -1
        if abs(det(ur5BodyJacobian(q))) < 1e-5
            finalerr = -1;
            disp('here is a sigularity')
            return
        end
    
        R_error = g_error(1:3,1:3);
        P_error = g_error(1:3,4);
        trace_R_error = R_error(1,1) + R_error(2,2) +R_error(3,3);
        theta= acos((trace_R_error-1)/2);
        p = sqrt((P_error(1))^2+(P_error(2))^2+(P_error(3))^2);
    
    
        if theta <= 5*pi/180 && p <= 0.02
            Tstep=0.1*Tstep;
        end
        %define the threshold (5cm, 15degree)
    
        if theta <= 0.5*pi/180 && p <= 0.001
            finalerr = theta*100;
            disp('convengence achieved')
            return
        end
        ur5.move_joints(q,5);
        pause(0.1)
    end
   

end





