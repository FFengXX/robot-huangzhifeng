% 加载数据
load('workspace_points.mat');  % 或 data = readmatrix('workspace_points.csv');

% 绘制3D散点图
figure;
scatter3(workspace_points(:,1), workspace_points(:,2), workspace_points(:,3), 1, 'b.');
xlabel('X'); ylabel('Y'); zlabel('Z');
title('机械臂工作空间');
axis equal;
grid on;

%直接绘制散点图（最简单）