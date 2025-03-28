close all;
clear;

ToDeg = 180/pi;
ToRad = pi/180;


th1=0;
th2=90;
th3=0;
d4=1000;
th5=0;
th6=0;
th7=-90;
DHfk7Dof_Lnya(th1,th2,th3,d4,th5,th6,th7,0);
view(134,12);
pause;
stp=15;


%% 微分运动测试雅可比矩阵
cla;

for  i=1:20
    DHfk7Dof_Lnya(th1,th2,th3,d4,th5,th6,th7,0);
    J=Jacobian7DoF_Ln(th1,th2,th3,d4,th5,th6,th7);
    dD=[10 0 0 0 0 0]';
    dth=pinv(J)*dD;
    th1=th1+dth(1)/pi*180;
    th2=th2+dth(2)/pi*180;
    th3=th3+dth(3)/pi*180;
    d4=d4+dth(4)/pi*180;
    th5=th5+dth(5)/pi*180;
    th6=th6+dth(6)/pi*180;
    th7=th7+dth(7)/pi*180;
end

%% 数值逆解测试
%% 输入目标末端位置及旋转矩阵
Tpos=[20,-30,-120]';     %期望位置
Trpy=[0,0,20]';     %期望位置


