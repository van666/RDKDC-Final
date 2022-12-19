clear all;
clc;
%% test for ur5Fwdkin 
% This the TEST for ur5FwdKin
% q: 6 × 1 joint space variable vector
ur5 = ur5_interface();
theta1 = 0;
theta2 = pi/4;
theta3 = 0;
theta4 = pi/4;
theta5 = 0;
theta6 = pi/4;

q = [theta1; theta2; theta3; theta4; theta5; theta6];

% Call function
gst = ur5FwdKin(q);
% Place Flame 
fwdKinToolFrame = tf_frame('base_link','fwdKinToolFrame',eye(4));
pause(1);
fwdKinToolFrame.move_frame('base_link',gst);
ur5.move_joints(q,10);
%- Read the new pose of FrameC 
gacReal=fwdKinToolFrame.read_frame('base_link');


%% test for bodyjacobian

q=[pi/2,pi/3,pi/4,pi/5,pi/2,pi/3]';
e=eye(6);
k=pi/100;
S=zeros(6);
gst=ur5FwdKin(q);
for i=1:6
        p=(ur5FwdKin(q+e(:,i)*k)-ur5FwdKin(q-e(:,i)*k))/(2*k);
        S(:,i)=unhat(inv(gst)*p);
end

j=ur5BodyJacobian(q);
err = norm(S-j);


%% test for manipulability

mu_sigmamin = zeros(16,1);
mu_detjac = zeros(16,1);
mu_invcond = zeros(16,1);

thelta3 = -pi/4:0.1:pi/4;
for i = 1:16
    thelta = thelta3(i);
    q = [pi/3; pi/2; thelta; pi/4; pi/4;pi/4];
    J = ur5BodyJacobian(q);

    mu_sigmamin(i) = manipulability("sigmamin", J);
    mu_detjac(i) = manipulability("detjac", J);
    mu_invcond(i) = manipulability("invcond", J);
    fprintf("thelta3 = %f\n mu for sigmamin = %f\n mu for detjac = %f\n mu for invcond = %f\n",thelta3,mu_sigmamin,mu_detjac,mu_invcond);
end

figure(1);
plot(thelta3,mu_sigmamin);
title('sigmamin');
xlabel('thelta3')
    
figure(2);
plot(thelta3,mu_detjac);
title('detjac');
xlabel('thelta3')

figure(3);
plot(thelta3,mu_invcond);
title('invcond');
xlabel('thelta3')

%% test for getXi
g1=expm([0,0,0,1;0,0,-1,2;0,1,0,3;0,0,0,0]);
%xi1=[1;2;3;1;0;0]
xi1=getXi(g1);
g2=expm([0,-1,0,3;1,0,0,2;0,0,0,3;0,0,0,0]);
%xi2=[3;2;3;0;0;1]
xi2=getXi(g2);
g3=expm([0,0,0,1;0,0,-1,1.5;0,1,0,1.5;0,0,0,0]);
%xi1=[1;1.5;1.5;0;-1;0]
xi3=getXi(g3);
g4=expm([0,-sqrt(3)/3,sqrt(3)/3,1;sqrt(3)/3,0,-sqrt(3)/3,1;-sqrt(3)/3,sqrt(3)/3,0,1;0,0,0,0]);
%xi1=[1;1;1;sqrt(3)/3;sqrt(3)/3;sqrt(3)/3]
xi4=getXi(g4);

%% test for ur5RRControl
ur5 = ur5_interface();
K = 0.7;


% q=[0,0,0,0,0,0]';
% ur5.move_joints(q,5);

qk = [1.1310; -0.6283; 1.0053; 0.5027; -0.1885; 0.4398];
gdesired = ur5FwdKin11(qk+[0.5;0;0;0;0;0]);
% gdesired = ur5FwdKin(qk);
ur5RRcontrol(qk,gdesired, K, ur5);
    

