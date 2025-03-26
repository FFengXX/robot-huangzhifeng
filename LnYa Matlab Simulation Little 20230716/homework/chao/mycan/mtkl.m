%%%%蒙特卡洛
clc; clear; close all;

ToDeg = 180/pi; % 弧度到角度的转换因子
ToRad = pi/180; % 角度到弧度的转换因子
UX = [1 0 0]';   % X轴单位向量
UY = [0 1 0]';   % Y轴单位向量
UZ = [0 0 1]';   % Z轴单位向量
% 定义伸缩关节的范围
z_min = 0;   % 最小伸缩长度
z_max = 500; % 最大伸缩长度


Link(1) = struct('name', 'Base', 'th', 0 * ToRad, 'dz', 0, 'dx', 0, 'alf', 0 * ToRad, 'az', UZ); % Base To 1
Link(2) = struct('name', 'J1', 'th', 0 * ToRad, 'dz', 500, 'dx', 0, 'alf', 90 * ToRad, 'az', UZ); % 1 TO 2
Link(3) = struct('name', 'J2', 'th', 0 * ToRad, 'dz', 0, 'dx', 900, 'alf', 0 * ToRad, 'az', UZ); % 2 TO 3
Link(4) = struct('name', 'J3', 'th', 0 * ToRad, 'dz', 0, 'dx', 500, 'alf', 90 * ToRad, 'az', UZ); % 3 TO 4  
Link(5) = struct('name', 'J4', 'th', 0 * ToRad, 'dz', 500, 'dx', 0, 'alf', 0 * ToRad, 'az', UZ); % 4 TO 5
Link(6) = struct('name', 'J5', 'th', 0 * ToRad, 'dz', 500, 'dx', 0, 'alf', -90 * ToRad, 'az', UZ); % 5 TO 6     伸缩关节
Link(7) = struct('name', 'J6', 'th', 0 * ToRad, 'dz', 600, 'dx', 0, 'alf', 90 * ToRad, 'az', UZ); % 6 TO 7
Link(8) = struct('name', 'J7', 'th', 0 * ToRad, 'dz', 400, 'dx', 0, 'alf', 0 * ToRad, 'az', UZ); % 7 TO 8

%%3.0（100，100，0）
% 定义机械臂的关节角度范围（单位：度）
th_min = [-180, -90, -100, -150, 0, -170, -180]; % 关节角度的下限
th_max = [180,  140,  130,  170, 0,  170,  180]; % 关节角度的上限

% 定义伸缩关节的长度范围
d_min = 0;   % 伸缩关节的最小长度
d_max = 500; % 伸缩关节的最大长度

N = 50000; % 蒙特卡洛采样点数（可调整）
workspace_points = zeros(N, 3); % 预分配数组以存储工作空间点

% 进行蒙特卡洛采样以生成工作空间点
for b = 1:N
    % 随机生成6个关节角度（在范围内均匀分布）
    th = th_min + (th_max - th_min) .* rand(1, 7);
    % 随机生成伸缩关节的长度（在范围内均匀分布）
    d5 = d_min + (d_max - d_min) * rand(1); % 第五个关节的长度

    % 更新Link结构中的关节参数
    for i = 1:7
        if i == 5
            Link(i+1).dz = d5; % 第五个关节是伸缩关节，更新 dz
        else
            Link(i+1).th = th(i) * ToRad; % 其他关节是旋转关节，更新 th
        end
    end

    % 计算每个连杆的正运动学
    for i = 1:8
        C = cos(Link(i).th); % 关节角度的余弦
        S = sin(Link(i).th); % 关节角度的正弦
        Ca = cos(Link(i).alf); % 连杆扭转角的余弦
        Sa = sin(Link(i).alf); % 连杆扭转角的正弦
        a = Link(i).dx; % 沿X轴的距离
        d = Link(i).dz; % 沿Z轴的距离

        % 计算变换矩阵的组成部分
        Link(i).n = [C, S, 0, 0]';
        Link(i).o = [-S*Ca, C*Ca, Sa, 0]';
        Link(i).a = [S*Sa, -C*Sa, Ca, 0]';
        Link(i).p = [a*C, a*S, d, 1]';

        % 存储旋转矩阵和变换矩阵
        Link(i).R = [Link(i).n(1:3), Link(i).o(1:3), Link(i).a(1:3)];
        Link(i).A = [Link(i).n, Link(i).o, Link(i).a, Link(i).p];
    end

    % 计算每个连杆末端的位置
    for i = 2:8
        Link(i).A = Link(i-1).A * Link(i).A; % 级联变换矩阵
        Link(i).p = Link(i).A(:, 4); % 提取末端执行器的位置
    end

    % 存储末端执行器的位置
    workspace_points(b, :) = Link(8).p(1:3)';
end

% 绘制机械臂的工作空间
figure;
hold on; % 保持当前图形
scatter3(workspace_points(:,1), workspace_points(:,2), workspace_points(:,3), 2, 'b', 'filled');
xlabel('X'); ylabel('Y'); zlabel('Z');
title('IRB 1300-12/1.4 - 蒙特卡洛采样工作空间');
grid on; axis equal;

% 定义盒子的尺寸
box_length = 2000; % 长
box_width = 1000;  % 宽
box_height = 2000; % 高
box_x_start = -1000; % 盒子的 x 轴起点
box_y_start = -500; % 盒子的 y 轴起点

% 定义盒子的顶点坐标
vertices = [
    box_x_start, box_y_start, 0;
    box_x_start + box_length, box_y_start, 0;
    box_x_start + box_length, box_y_start + box_width, 0;
    box_x_start, box_y_start + box_width, 0;
    box_x_start, box_y_start, box_height;
    box_x_start + box_length, box_y_start, box_height;
    box_x_start + box_length, box_y_start + box_width, box_height;
    box_x_start, box_y_start + box_width, box_height;
];

% 定义盒子的面（每个面由4个顶点组成）
faces = [
    1, 2, 3, 4; % 底面
    5, 6, 7, 8; % 顶面
    1, 2, 6, 5; % 前面
    2, 3, 7, 6; % 右面
    3, 4, 8, 7; % 后面
    4, 1, 5, 8; % 左面
];

% 绘制盒子
patch('Vertices', vertices, 'Faces', faces, 'FaceColor', 'none', 'EdgeColor', 'r', 'LineWidth', 1.5);

% 生成10个小球
num_spheres = 10; % 小球数量
sphere_radius = 50; % 小球的半径
[x, y, z] = sphere(20); % 生成小球的几何数据

% 随机生成小球的位置
for i = 1:num_spheres
    % 随机生成小球中心的位置
    center_x = box_x_start + rand() * box_length; % 调整 x 坐标
    center_y = box_y_start + rand() * box_width;  % 调整 y 坐标
    center_z = rand() * box_height;

    % 绘制小球
    surf(x * sphere_radius + center_x, ...
         y * sphere_radius + center_y, ...
         z * sphere_radius + center_z, ...
         'EdgeColor', 'none', 'FaceColor', 'g'); % 绿色小球
end

% 设置图形属性
xlabel('X');
ylabel('Y');
zlabel('Z');
title('工作空间');
axis equal;
grid on;
view(3); % 设置为 3D 视角
hold off;