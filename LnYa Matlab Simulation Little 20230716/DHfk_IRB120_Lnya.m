function DHfk_IRB120_Lnya(th1,th2,th3,th4,th5,th6,fcla)
% 关闭所有图形窗口（注释掉的原始代码）
% close all

global Link  % 声明全局变量 Link，用于存储机械臂连杆参数

Build_IRB120_Lnya;  % 加载 IRB120 机械臂的 DH 参数和模型配置

radius    = 25;  % 关节圆柱体的半径（单位：毫米）
len       = 60;  % 关节圆柱体的长度（单位：毫米）
joint_col = 0;   % 关节颜色（0 表示默认颜色）

% 在原点绘制红色圆圈标记
plot3(0,0,0,'ro'); 

% 更新各关节角度（将输入的角度转换为弧度并叠加到初始角度）
Link(2).th = Link(2).th + th1*pi/180;  % 关节1角度更新
Link(3).th = Link(3).th + th2*pi/180;  % 关节2角度更新
Link(4).th = Link(4).th + th3*pi/180;  % 关节3角度更新
Link(5).th = Link(5).th + th4*pi/180;  % 关节4角度更新
Link(6).th = Link(6).th + th5*pi/180;  % 关节5角度更新
Link(7).th = Link(7).th + th6*pi/180;  % 关节6角度更新（初始位置）

% 另一种直接赋值关节角度的方式（注释掉的代码）
% Link(2).th = th1*pi/180;
% ... 

p0 = [0,0,0]';  % 定义初始原点坐标

% 遍历所有连杆计算 DH 变换矩阵
for i = 1:9
    Matrix_DH_Ln(i);  % 计算第i个连杆的 DH 矩阵
end

% 遍历所有连杆计算位姿并绘制模型
for i = 2:9
    % 计算累积变换矩阵（相对于基座）
    Link(i).A = Link(i-1).A * Link(i).A;
    
    % 提取位置向量（变换矩阵第4列）
    Link(i).p = Link(i).A(:,4);  
    
    % 提取方向向量（变换矩阵的前三列）
    Link(i).n = Link(i).A(:,1);  % X轴方向
    Link(i).o = Link(i).A(:,2);  % Y轴方向
    Link(i).a = Link(i).A(:,3);  % Z轴方向
    
    % 构建旋转矩阵
    Link(i).R = [Link(i).n(1:3), Link(i).o(1:3), Link(i).a(1:3)];
    
    % 绘制连杆（蓝色线段连接相邻关节）
    Connect3D(Link(i-1).p, Link(i).p, 'b', 2); 
    hold on;
    
    % 标记当前关节位置（红色叉号）
    plot3(Link(i).p(1), Link(i).p(2), Link(i).p(3), 'rx'); 
    hold on;
    
    % 绘制前7个关节的圆柱体（表示物理结构）
    if i <= 7
        DrawCylinder(Link(i-1).p, Link(i-1).R * Link(i).az, radius, len, joint_col); 
        hold on;
    end 
end

% 设置坐标轴范围和标签
axis([-700,700,-700,700,-400,1000]);  % 设置三维坐标系范围
xlabel('x');  % X轴标签
ylabel('y');  % Y轴标签
zlabel('z');  % Z轴标签
grid on;      % 显示网格
drawnow;      % 立即刷新图形

% 根据输入参数决定是否清除图形
if(fcla)
    cla;  % 清除当前坐标系内容
end