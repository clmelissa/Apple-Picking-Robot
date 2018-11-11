close all;
clear;

% L1&L2 refers to the length of the 2 arms in meter
L1 = 0.5;
L2 = 0.5;

%desired position
desiredX = 0.7;
desiredY = 0.5;
desiredPos = [desiredX desiredY 0];

%elbow angle
cos_e = (desiredX^2+desiredY^2-L1^2-L2^2) / (2*L1*L2);
E = acos(cos_e);

cos_q = (desiredX^2+desiredY^2+L1^2-L2^2) / (2*L1*sqrt(desiredX^2+desiredY^2));
Q = acos(cos_q);

s = atan2(desiredY,desiredY) - Q;

%Path visualization
robot = ScaraArmInit(L1,L2);
showdetails(robot)

q0 = homeConfiguration(robot);
ndof = 2;

ik = robotics.InverseKinematics('RigidBodyTree', robot);
weights = [0, 0, 0, 1, 1, 0];
endEffector = 'tool';

% Solve for the configuration satisfying the desired end effector
% position
qSol = ik(endEffector,trvec2tform(desiredPos),weights,[0;0]);

% Plot the robot position
figure
show(robot,qSol);
view(2)
ax = gca;
ax.Projection = 'orthographic';
hold on
plot(desiredX,desiredY,'r*') 
textString = sprintf('( %.2f,  %.2f)', desiredX, desiredY);
text(desiredX+0.3, desiredY+0.1, textString);
axis([-0.0 2.0 -1.0 1.0])


