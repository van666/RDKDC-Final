function Invkin(qs,qt,ur5)

gs=ur5FwdKin(qs);
gt=ur5FwdKin(qt);

d=0.1;
t=10;
gsabove=gs;
gsabove(3,4)=gsabove(3,4)+d;
gtabove=gt;
gtabove(3,4)=gtabove(3,4)+d;
%step 0.home configuration
ur5.move_joints(ur5.home,t);
pause(t)

%step 1
q1above=ur5InvKin(gsabove);
q1above_chosen=q1above(:,1);
ur5.move_joints(q1above_chosen,t);
pause(t/4);
display("step 1 ");

%step2
q1=ur5InvKin(gs);
q1_chosen=q1(:,1);
ur5.move_joints(q1_chosen,t);
pause(t/4);
display("step 2 ");

%step3
ur5.move_joints(q1above_chosen,t);
pause(t/4);
% ur5.move_joints(ur5.home,t);
% pause(t/4);
display("step 3 ");

%step4
q2above=ur5InvKin(gtabove);
q2above_chosen=q2above(:,1);
ur5.move_joints(q2above_chosen,t/2);
pause(t/4);
display("step 4 ");

%step5
q2=ur5InvKin(gt);
q2_chosen=q2(:,1);
ur5.move_joints(q2_chosen,t/2);
ur5.move_joints(q2above_chosen,t/2);
display("step 5 ");

end
 
