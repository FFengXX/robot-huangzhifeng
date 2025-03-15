function pic=DHfk6Dof_Lnya(th1,th2,d3,th4,th5,th6,fcla)
% close all
%disp(['th1: ', num2str(th1)]);
%disp(['th2: ', num2str(th2)]);
%disp(['th3: ', num2str(th3)]);
%disp(['th4: ', num2str(th4)]);
%disp(['th5: ', num2str(th5)]);
%disp(['th6: ', num2str(th6)]);

global Link

Build_6DOFRobot_Lnya;
radius = 40;
len = 100;
joint_col = 0;

plot3(0,0,0,'ro');

Link(2).th=th1*pi/180;
Link(3).th=th2*pi/180;
Link(4).dx=d3;
Link(5).th=th4*pi/180;
Link(6).th=th5*pi/180;
Link(7).th=th6*pi/180;

% Link(2).th=Link(2).th+th1*pi/180;
% Link(3).th=Link(3).th+th2*pi/180;
% Link(4).th=Link(4).th+th3*pi/180;
% Link(5).th=Link(5).th+th4*pi/180;
% Link(6).th=Link(6).th+th5*pi/180;
p0=[0,0,0]';

for i=1:8
    Matrix_DH_Ln(i);
end

for i=2:8
    Link(i).A=Link(i-1).A*Link(i).A;
    Link(i).p= Link(i).A(:,4);
    Link(i).n= Link(i).A(:,1);
    Link(i).o= Link(i).A(:,2);
    Link(i).a= Link(i).A(:,3);
    Link(i).R=[Link(i).n(1:3),Link(i).o(1:3),Link(i).a(1:3)];


    Connect3D(Link(i-1).p,Link(i).p,'b',2); hold on;% 连接相邻连杆的线段
    if i<8
        DrawCylinder(Link(i-1).p, Link(i-1).R * Link(i).az, radius,len, joint_col); hold on; % 绘制关节圆柱体
    end
end

grid on;
view(134,12);
% axis([-2400,2400,-2000,2000,-2300,2300]);
axis([-3000,3000,-3000,3000,-3000,3000]);
xlabel('x');
ylabel('y');
zlabel('z');
drawnow;
pic=getframe;
if(fcla)
cla;
end