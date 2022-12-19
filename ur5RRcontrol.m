function [finalerr] =ur5RRcontrol(gdesired, K, ur5)
    % initialize Tstep
    Tstep = 0.1;
    % set the parameters to calculate Tstep
    m=0;
    beta=0.9;
    
    q=ur5.get_current_joints();
    gd_inv = inv(gdesired);

    %set the frame of gdesired
    desiredFrame = tf_frame('base_link','desiredFrame',eye(4));
    desiredFrame.move_frame('base_link',gdesired);

    % begin the loop to simulate the track
    while 1
        gst = ur5FwdKin(q);
        g_error = gd_inv * gst;
        s = getXi(g_error);
        
        %calculate the intended angular for this iteration
        q=q-K*Tstep*inv(ur5BodyJacobian(q))*s;
    
        % check singulariries; if failure, return -1
        ability1 = manipulability("detjac",ur5BodyJacobian(q)) 
        ability2 = manipulability("sigmamin",ur5BodyJacobian(q)) 
        ability3 = manipulability("invcond",ur5BodyJacobian(q)) 
        if abs(ability1) < 1e-5 ||abs(ability2) < 1e-5||abs(ability3) < 1e-5
            finalerr = -1;
            disp('here is a sigularity')
            return
        end
        
        %calculate the error of angular and position
        R_error = g_error(1:3,1:3);
        P_error = g_error(1:3,4);
        trace_R_error = R_error(1,1) + R_error(2,2) +R_error(3,3);
        theta= acos((trace_R_error-1)/2);
        p = sqrt((P_error(1))^2+(P_error(2))^2+(P_error(3))^2);
    
    
%         if theta <= 5*pi/180 && p <= 0.02
%             Tstep=0.1*Tstep;
%         end

        %define the threshold (5cm, 15degree)
        if theta <= 0.5*pi/180 && p <= 0.001
            finalerr = theta*100;
            disp('convengence achieved')
            return
        end

        %move the ur5
        ur5.move_joints(q,5);
        pause(0.1)

        %calculate Tstep for next iteration
        m = beta*m + (1-beta)*P_error;
        Tstep=Tstep*m;
    end
   

end





