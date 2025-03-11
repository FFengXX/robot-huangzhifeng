% 创建 alphaShape 对象（自动拟合非凸形状）

alpha= 100; %控制表面拟合紧密度（值越大表面越光滑，越小越贴合点云）。
shp = alphaShape(workspace_points(:,1), workspace_points(:,2), workspace_points(:,3), alpha);

% 绘制表面
figure;
plot(shp, 'FaceColor', 'magenta', 'EdgeColor', 'none');
xlabel('X'); ylabel('Y'); zlabel('Z');
title('工作空间 AlphaShape');
axis equal;

%使用 alphaShape 生成精确表面