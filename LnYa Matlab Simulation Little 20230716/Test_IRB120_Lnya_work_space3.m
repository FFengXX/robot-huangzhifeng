% 计算点云的凸包（表面三角网格）
k = convhull(workspace_points(:,1), workspace_points(:,2), workspace_points(:,3));

% 绘制3D凸包
figure;
trisurf(k, workspace_points(:,1), workspace_points(:,2), workspace_points(:,3), 'FaceColor', 'cyan', 'EdgeColor', 'none');
xlabel('X'); ylabel('Y'); zlabel('Z');
title('工作空间凸包');
axis equal;
light; lighting gouraud; % 添加光照效果

%生成凸包（表面模型）​