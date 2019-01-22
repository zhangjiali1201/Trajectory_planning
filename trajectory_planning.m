clear;
clc;
L1 = Link('alpha', pi/2, 'a',288, 'd',0);%定义连杆
L2 = Link('alpha', 0, 'a',1370, 'd',288, 'offset',pi/2);
L3 = Link('alpha', pi/2, 'a',0, 'd',-288);
L4 = Link('alpha', 0, 'a',0, 'd',288);
L5 = Link('alpha', pi/2, 'a',0, 'd',1000);
L6 = Link('alpha', 0, 'a',228, 'd',228, 'offset',pi/2);
robot=SerialLink([L1,L2,L3,L4,L5,L6]);   %建立连杆
robot.name ='15123098';     %名字


init_ang=[0 0 0 0 0 0];%p1
targ_ang=[pi/6,-pi/3,pi/9,0,0,0];%p2
targ_ang2=[0,0,-pi/4,0,0,0];%p3
targ_ang3=[pi/9,pi/9,-pi/6,0,0,0];%p4
targ_ang4=[pi/9,-pi/2,pi/9,0,0,0];%p5
targ_ang5=[5*pi/36,-5*pi/12,pi/9,0,0,0];%p6
targ_ang6=[-pi/9,-pi/4,0,0,0,0];%p7
targ_ang7=[-pi/10,-pi/2.5,pi/18,0,0,0];%p8
targ_ang8=[-pi/6,-pi/18,0,0,0,0];%p9
targ_ang9=[-pi/7.2,-pi/1.9,pi/18,0,0,0];%p10
step=10;%运动速度
[q,qd,qdd] = jtraj(init_ang, targ_ang, step);
[q2,qd,qdd] = jtraj(targ_ang, targ_ang2, step);
[q3,qd,qdd] = jtraj(targ_ang2, targ_ang3, step);
[q4,qd,qdd] = jtraj(targ_ang3, targ_ang4, step);
[q5,qd,qdd] = jtraj(targ_ang4, targ_ang5, step);
[q6,qd,qdd] = jtraj(targ_ang5, targ_ang6, step);
[q7,qd,qdd] = jtraj(targ_ang6, targ_ang7, step);
[q8,qd,qdd] = jtraj(targ_ang7, targ_ang8, step);
[q9,qd,qdd] = jtraj(targ_ang8, targ_ang9, step);%连线
subplot(1,2,1);%图像位置
robot.plot(q);
robot.plot(q2);
robot.plot(q3);
robot.plot(q4);
robot.plot(q5);
robot.plot(q6);
robot.plot(q7);
robot.plot(q8);
robot.plot(q9);%运动

%"利"字轨迹
title('利 轨迹');
p1=robot.fkine(init_ang)
p2=robot.fkine(targ_ang)
Tc=ctraj(p1, p2, step);
Tjtraj=transl(Tc);
subplot(1,2,2);
plot2(Tjtraj,'r');
hold on;%撇
grid on;

p2=robot.fkine(targ_ang)
p3=robot.fkine(targ_ang2)
Tc=ctraj(p2, p3, step);
Tjtraj=transl(Tc);
subplot(1,2,2);
plot2(Tjtraj,'r');
hold on;%横
grid on;

p3=robot.fkine(targ_ang2)
p4=robot.fkine(targ_ang3)
Tc=ctraj(p3, p4, step);
Tjtraj=transl(Tc);
subplot(1,2,2);
plot2(Tjtraj,'r');
hold on;%斜上
grid on;

p4=robot.fkine(targ_ang3)
p5=robot.fkine(targ_ang4)
Tc=ctraj(p4, p5, step);
Tjtraj=transl(Tc);
subplot(1,2,2);
plot2(Tjtraj,'r');
hold on;%竖
grid on;

p5=robot.fkine(targ_ang4)
p6=robot.fkine(targ_ang5)
Tc=ctraj(p5, p6, step);
Tjtraj=transl(Tc);
subplot(1,2,2);
plot2(Tjtraj,'r');
hold on;%斜上
grid on;

p6=robot.fkine(targ_ang5)
p7=robot.fkine(targ_ang6)
Tc=ctraj(p6, p7, step);
Tjtraj=transl(Tc);
subplot(1,2,2);
plot2(Tjtraj,'r');
hold on;%斜上
grid on;

p7=robot.fkine(targ_ang6)
p8=robot.fkine(targ_ang7)
Tc=ctraj(p7, p8, step);
Tjtraj=transl(Tc);
subplot(1,2,2);
plot2(Tjtraj,'r');
hold on;%短竖
grid on;

p8=robot.fkine(targ_ang7)
p9=robot.fkine(targ_ang8)
Tc=ctraj(p8, p9, step);
Tjtraj=transl(Tc);
subplot(1,2,2);
plot2(Tjtraj,'r');
hold on;%斜上
grid on;

p9=robot.fkine(targ_ang8)
p10=robot.fkine(targ_ang9)
Tc=ctraj(p9, p10, step);
Tjtraj=transl(Tc);
subplot(1,2,2);
plot2(Tjtraj,'r');
hold on;%长竖
grid on;