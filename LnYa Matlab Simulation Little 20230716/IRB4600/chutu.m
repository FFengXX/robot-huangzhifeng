close all;
% A(1:90)=struct('cdata',[],'colormap',[]);
clear;

global Link
% Build_6DOFRobot_Lnya;
% D-H参数（单位：米）
a = [377, 1145, 200, 0, 0, 0]; % 连杆长度
d = [780, 0, 0, 1462.5, 0, 220]; % 连杆偏移
alpha = [pi/2, 0, pi/2, -pi/2, pi/2, 0]; % 连杆扭转角
% theta = 
% 关节角度范围（度）
theta_ranges = [ 
[-170, 170];
[-65, 85];
[-70, 180];
[-300, 300];
[-130, 130];
[-360, 360];
];

% 蒙特卡洛采样
N = 100000; % 采样点数
points = zeros(N, 3); % 存储末端坐标

for k = 1:N
% 生成随机关节角度（弧度）
theta = theta_ranges(:,1) + (theta_ranges(:,2)-theta_ranges(:,1)) .* rand(6,1);
theta = deg2rad(theta);
% 正运动学计算
T = eye(4); % 初始变换矩阵
for i = 1:6
ct = cos(theta(i));
st = sin(theta(i));
ca = cos(alpha(i));
sa = sin(alpha(i));
Ti = [ ct, -st*ca, st*sa, a(i)*ct;
st, ct*ca, -ct*sa, a(i)*st;
0, sa, ca, d(i);
0, 0, 0, 1];
T = T * Ti; % 级联变换矩阵
end
points(k, :) = T(1:3, 4)'; % 提取末端坐标（单位：米）
end

% 绘制工作空间
figure;
scatter3(points(:,1), points(:,2), points(:,3), 1, 'b.', 'MarkerEdgeAlpha', 0.3);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('IRB4600工作空间点云');
axis equal;
grid on;