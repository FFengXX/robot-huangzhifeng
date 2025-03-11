% 加载工作空间点云
load('workspace_points.mat');

% 绘制工作空间（灰色点云）
figure;
scatter3(workspace_points(:,1), workspace_points(:,2), workspace_points(:,3), 1, [0.5,0.5,0.5]);
hold on;

% 绘制机械臂在某个姿态下的模型（示例角度）
DHfk_IRB120_Lnya(30, -45, 60, 0, 90, 0, false);

% 美化图形
xlabel('X'); ylabel('Y'); zlabel('Z');
title('工作空间与机械臂模型');
axis equal;
grid on;

%结合机械臂模型显示