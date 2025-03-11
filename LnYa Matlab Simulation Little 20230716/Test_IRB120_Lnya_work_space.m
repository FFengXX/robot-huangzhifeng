% generate_workspace.m
% 生成机械臂工作空间点云并保存为文件

%% 清空环境
clc;
clear all;
close all;

%% 定义关节参数
global Link;  % 声明全局变量（确保已加载机械臂模型）
joint_limits = [     % 关节角度范围（单位：度）
    0,  165;   % 关节1
    -110,  110;   % 关节2
    -90,  70;   % 关节3
    -160,  160;   % 关节4
    -120,  120;   % 关节5
    -400,  400;   % 关节6
];
step = 15;      % 遍历步长（越小精度越高，但计算量指数级增长）

%% 初始化机械臂模型（需确保 Build_IRB120_Lnya 函数可用）
Build_IRB120_Lnya;

%% 遍历关节角度生成工作空间点云
angles = cell(6, 1);  % 存储各关节角度序列
for i = 1:6
    angles{i} = joint_limits(i,1):step:joint_limits(i,2);
end

% 预分配内存（优化性能）
num_points = prod(cellfun(@numel, angles(1:3)));  % 仅遍历前3个关节
workspace_points = zeros(num_points, 3);
idx = 1;  % 索引计数器

% 遍历前3个关节（后3关节固定为0度）
disp('正在生成工作空间点云...');
for th1 = angles{1}
    for th2 = angles{2}
        for th3 = angles{3}
            % 计算机械臂正运动学
            DHfk_IRB120_Lnya(th1, th2, th3, 0, 0, 0, true);
            
            % 获取末端执行器位置（假设末端在 Link(9).p）
            end_effector_pos = Link(9).p(1:3);
            
            % 存储位置
            workspace_points(idx, :) = end_effector_pos';
            idx = idx + 1;
        end
    end
end
disp('工作空间点云生成完成！');

%% 保存数据
% 保存为MAT文件（MATLAB专用，支持快速加载）
save('workspace_points.mat', 'workspace_points');

% 保存为CSV文件（通用格式，可在Excel/Python中打开）
writematrix(workspace_points, 'workspace_points.csv');

%% 可视化验证
figure;
scatter3(workspace_points(:,1), workspace_points(:,2), workspace_points(:,3), 1, 'b.');
xlabel('X'); ylabel('Y'); zlabel('Z');
title('机械臂工作空间点云');
axis equal;
grid on;
