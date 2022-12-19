clear;
clc;
ur5=ur5_interface();
%start position:
start=input("Please press enter to record the start position");
qs=ur5.get_current_joints();
%target position:
target=input("Please press enter to record the target position");
qt=ur5.get_current_joints();

%choose mode
mode=input("Please choose mode:1 represents Invkin, 2 represents RRInv, 3 represents RRTran");
while (mode~=1 && mode~=2 && mode~=3)
    mode=input("PLease enter 1,2 or 3:1 represents Invkin, 2 represents RRInv, 3 represents RRTran");
end
if mode==1
    Invkin(qs,qt,ur5);
elseif mode==2
        ur5RRcontrol()
elseif mode==3
        TransposeJacobian()
end