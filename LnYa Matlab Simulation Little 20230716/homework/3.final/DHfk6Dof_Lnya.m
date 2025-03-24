function ret=DHfk6Dof_Lnya(th1,th2,th3,th4,d5,th6,th7,fcla)
global Link

Build_6DOFRobot_Lnya;
radius = 40;
len = 100;
joint_col = 0;

%plot3(0,0,0,'ro');红点

Link(2).th=th1*pi/180;
Link(3).th=th2*pi/180;
Link(4).th=th3*pi/180;
Link(5).th=th4*pi/180;
Link(6).th=d5;
Link(7).th=th6*pi/180;
Link(8).th=th7*pi/180;

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


    Connect3D(Link(i-1).p,Link(i).p,'r',2); hold on;% 连接相邻连杆的线段
    if i<9
        DrawCylinder(Link(i-1).p, Link(i-1).R * Link(i).az, radius,len, joint_col); hold on; % 绘制关节圆柱体
    end
end

% 在绘制完所有连杆后添加以下代码（在for循环之后）：
% Link(8).p 是末端执行器的位置（4x1齐次坐标）
end_effector_pos = Link(8).p(1:3);  % 提取x,y,z坐标
%plot3(end_effector_pos(1), end_effector_pos(2), end_effector_pos(3), 'ro');

grid on;
view(134,12);
% axis([-2400,2400,-2000,2000,-2300,2300]);
axis([-3000,3000,-3000,3000,-3000,3000]);
xlabel('x');
ylabel('y');
zlabel('z');
drawnow;

ret=end_effector_pos;
if(fcla)
cla;
end