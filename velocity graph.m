clear L
%            theta    d           a       alpha
L(1) = Link([0        0.4         0.18    pi/2]);
L(2) = Link([0        0.135       0.60    pi]);
L(3) = Link([0        0.135       0.12   -pi/2]);
L(4) = Link([0        0.62        0       pi/2]);
L(5) = Link([0        0           0      -pi/2]);
L(6) = Link([0        0           0       0]);
KR5=SerialLink(L, 'name', 'Kuka KR5');
KR5.tool=transl(0,0,0.05);%The fingers of the robot gripper is 50mm
KR5.ikineType = 'kr5';
KR5.model3d = 'KUKA/KR5_arc';


T0=[1 0 0 0.5;
    0 1 0 0.5;
    0 0 1 0.5;
    0 0 0 1;];
IK0=KR5.ikine6s(T0,'lun');
disp(IK0);
FK0=KR5.fkine(IK0);
disp(FK0);
%Suppose the base point is at(0,0,0)
%Suppose the robot is at this position due to T0 at first.Each of the coodinate of gripper and base is on the same direction.
%The position of the gripper is (0.5 0.5 0.5)
T1=transl(-0.125,-0.5,-0.45)*troty(pi) ;%First the screw is at (0.375 0 0.05)
disp(T1);
IK1=KR5.ikine6s(T1,'lun');%KR5 inverse kinematics
disp(IK1);
FK1=KR5.fkine(IK1);
disp(FK1);
T2=transl(-0.375,0.2,-0.025);%The screw is moved to (0 0.2 0.025) from (0.375 0 0.05) and the orientation is not changed.
IK2=KR5.ikine6s(T2,'lun');
disp(IK2);
FK2=KR5.fkine(IK2);
disp(FK2);

t1=[0:0.05:2]';
t2=[2:0.05:4]';
[q1,qd1,qdd1]=jtraj(IK0,IK1,t1);
[q2,qd2,qdd2]=jtraj(IK1,IK2,t2);
subplot(2,2,1)
plot(t1,q1);xlabel('time(s)'),ylabel('theta(rad)');
subplot(2,2,2)
plot(t2,q2);xlabel('time(s)'),ylabel('theta(rad)');
subplot(2,2,3)
plot(t1,qd1);xlabel('time(s)'),ylabel('velocity(m/s)');
subplot(2,2,4)
plot(t2,qd2);xlabel('time(s)'),ylabel('velocity(m/s)');