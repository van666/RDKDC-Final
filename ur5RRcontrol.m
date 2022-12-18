function [finalerr] =ur5RRcontrol(q, gdesired, K, ur5)
   
    Tstep = 0.001;
    iteration = 100;
%     q = [pi/4;0;pi/3;pi/4;0;pi/4];

%     gd_inv=inv(gdesired);

    desiredFrame = tf_frame('base_link','desiredFrame',eye(4));
    desiredFrame.move_frame('base_link',gdesired);

    for i = 0:Tstep:iteration*Tstep
        gst = ur5FwdKin(q);
        g_error = inv(gdesired)*gst;
        s = getXi(g_error);

        q2=q-K*Tstep*inv(ur5BodyJacobian(q))*s;

        % check singulariries; if fature, returen -1
        if abs(det(ur5BodyJacobian(q))) < 0.001
            finalerr = -1;
            return
        end

        %get vk and wk 
        R_error = g_error(1:3,1:3);
        P_error = g_error(1:3,4);
        trace_R_error = R_error(1,1) + R_error(2,2) +R_error(3,3);
        wk = acos((trace_R_error-1)/2);
        vk = P_error(1)*P_error(2)*P_error(3);

        %define the threshold (5cm, 15degree)
        if wk <= 15*pi/180 && vk <= 0.05
            finalerr = vk*100;
            return
        end
        ur5.move_joints(q,5);
        pause(1)  
     end

end





