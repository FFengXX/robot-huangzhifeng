% 生成网格模型（以凸包为例）
k = convhull(workspace_points(:,1), workspace_points(:,2), workspace_points(:,3));

% 导出为STL文件
vertices = workspace_points;
faces = k;
stlwrite('workspace.stl', faces, vertices);

% 提示信息
disp('STL文件已保存为 workspace.stl');

%导出为 STL 文件（用于3D打印或其他软件）​