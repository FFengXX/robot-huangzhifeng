%%%%%%%%%%%%运行rrt

% 开始关节
start_q = [0, 90, 0, 0, 0, 0, 0];  

radius = 40;
fileID = fopen('障碍物.txt', 'r');   % 打开文件
obstacles = textscan(fileID, '%f %f %f', 'Delimiter', ',');  % 按列读取数字，并指定分隔符为逗号
fclose(fileID);  % 关闭文件

% 将读取的数据转换为矩阵
obstacles = cell2mat(obstacles);
%%%%%%%%%%%%%%%%%%%%6.0
% 读取小球位置
% 读取小球位置  小球消失成功
qiuwzhi = readArrayFromFile('障碍物.txt');

% 定义盒子的尺寸和起始位置
box_length = 1000;        % 长 = max(x) - min(x) = 1532 - 532 = 1000
box_width  = 2000;        % 宽 = max(y) - min(y) = 677 - (-1323) = 2000
box_height = 2000;        % 高 = max(z) - min(z) = 2139 - 139 = 2000
box_x_start = 532;        % 盒子的 x 轴起点 (min(x))
box_y_start = -1323;      % 盒子的 y 轴起点 (min(y))
box_z_start = 139;        % 盒子的 z 轴起点 (min(z))
% 定义球的半径
sphere_radius = 50;

% 创建图形窗口
figure;
hold on;
grid on;
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('机械臂轨迹');
view(3);

% 定义盒子的顶点
vertices = [
    box_x_start, box_y_start, box_z_start;
    box_x_start + box_length, box_y_start, box_z_start;
    box_x_start + box_length, box_y_start + box_width, box_z_start;
    box_x_start, box_y_start + box_width, box_z_start;
    box_x_start, box_y_start, box_z_start + box_height;
    box_x_start + box_length, box_y_start, box_z_start + box_height;
    box_x_start + box_length, box_y_start + box_width, box_z_start + box_height;
    box_x_start, box_y_start + box_width, box_z_start + box_height;
];

% 定义盒子的面
faces = [
    1, 2, 3, 4; % 底面
    5, 6, 7, 8; % 顶面
    1, 2, 6, 5; % 前面
    2, 3, 7, 6; % 右面
    3, 4, 8, 7; % 后面
    4, 1, 5, 8  % 左面
];

% 绘制盒子
patch('Vertices', vertices, 'Faces', faces, 'FaceColor', 'none', 'FaceAlpha', 1, 'EdgeColor', 'blue');

% 绘制小球
[x, y, z] = sphere(20); % 生成小球的几何数据
ball_handles = []; % 存储小球的图形句柄
for i = 1:size(qiuwzhi, 1)
    % 获取小球的位置
    center_x = qiuwzhi(i, 1);
    center_y = qiuwzhi(i, 2);
    center_z = qiuwzhi(i, 3);

    % 绘制小球并存储句柄
    ball_handles(i) = surf(x * sphere_radius + center_x, ...
                           y * sphere_radius + center_y, ...
                           z * sphere_radius + center_z, ...
                           'EdgeColor', 'y', 'FaceColor', 'y');
end

% 设置图形属性
axis([box_x_start, box_x_start + box_length, ...
      box_y_start, box_y_start + box_width, ...
      box_z_start, box_z_start + box_height]);

% 读取目标点
filename = '收敛角度.txt';
nijiao = readArrayFromFile(filename);
disp(nijiao);

% 定义接近小球的阈值
threshold = 10; % 当机械臂末端与小球的距离小于此值时，认为到达小球位置

% 逐行调用 RRT4 函数
for i = 1:size(nijiao, 1)
    % 获取第 i 行作为目标点
    goal_q = nijiao(i, :);
    
    % 调用 RRT4 函数
    [path] = Path_planning(start_q, goal_q, obstacles);
    founditerations = [];
    
    % 正向遍历路径中的每个节点（从起始点到目标点）
    for j = 1:size(path, 1)
        target_angles = path(j, :);
        
        % 更新关节角度
        th1 = target_angles(1);
        th2 = target_angles(2);
        th3 = target_angles(3);
        th4 = target_angles(4);
        th5 = target_angles(5);
        th6 = target_angles(6);
        th7 = target_angles(7);  
        
        % 使用运动学模型计算机械臂末端执行器位置
        points = DHfk7Dof_Lnya2(th1, th2, th3, th4, th5, th6, th7,1);  
        end_effector_x = points(1);
        end_effector_y = points(2);
        end_effector_z = points(3);

        
        % 重新绘制小球和盒子
        cla; % 清除当前窗口的所有图形
        patch('Vertices', vertices, 'Faces', faces, 'FaceColor', 'none', 'FaceAlpha', 0.1, 'EdgeColor', 'blue');
        
        % 检查机械臂末端是否接近小球
        for k = size(qiuwzhi, 1):-1:1
            center_x = qiuwzhi(k, 1);
            center_y = qiuwzhi(k, 2);
            center_z = qiuwzhi(k, 3);
            
            % 计算机械臂末端与小球的距离
            distance = sqrt((end_effector_x - center_x)^2 + ...
                            (end_effector_y - center_y)^2 + ...
                            (end_effector_z - center_z)^2);
            
            % 如果距离小于阈值，移除小球
            if distance < threshold
                if ishandle(ball_handles(k)) % 检查句柄是否有效
                    delete(ball_handles(k)); % 删除小球的图形
                end
                qiuwzhi(k, :) = []; % 从矩阵中移除小球的位置
                ball_handles(k) = []; % 从句柄数组中移除小球的句柄
                drawnow; % 立即更新图形
            else
                % 否则，重新绘制小球
                surf(x * sphere_radius + center_x, ...
                     y * sphere_radius + center_y, ...
                     z * sphere_radius + center_z, ...
                     'EdgeColor', 'y', 'FaceColor', 'y');
            end
        end
    end

    % 回到原点，逆向遍历路径
    for j = size(path, 1):-1:1
        target_angles = path(j, :);
        
        % 更新关节角度
        th1 = target_angles(1);
        th2 = target_angles(2);
        th3 = target_angles(3);
        th4 = target_angles(4);
        th5 = target_angles(5);
        th6 = target_angles(6);
        th7 = target_angles(7);  
        
        % 使用运动学模型计算机械臂末端执行器位置
        points = DHfk7Dof_Lnya2(th1, th2, th3, th4, th5, th6, th7,1);  
        end_effector_x = points(1);
        end_effector_y = points(2);
        end_effector_z = points(3);
        % 重新绘制小球和盒子
        cla; % 清除当前窗口的所有图形
        patch('Vertices', vertices, 'Faces', faces, 'FaceColor', 'none', 'FaceAlpha', 0.1, 'EdgeColor', 'blue');
        
        % 检查机械臂末端是否接近小球
        for k = size(qiuwzhi, 1):-1:1
            center_x = qiuwzhi(k, 1);
            center_y = qiuwzhi(k, 2);
            center_z = qiuwzhi(k, 3);
            
            % 计算机械臂末端与小球的距离
            distance = sqrt((end_effector_x - center_x)^2 + ...
                            (end_effector_y - center_y)^2 + ...
                            (end_effector_z - center_z)^2);
            
            % 如果距离小于阈值，移除小球
            if distance < threshold
                if ishandle(ball_handles(k)) % 检查句柄是否有效
                    delete(ball_handles(k)); % 删除小球的图形
                end
                qiuwzhi(k, :) = []; % 从矩阵中移除小球的位置
                ball_handles(k) = []; % 从句柄数组中移除小球的句柄
                drawnow; % 立即更新图形
            else
                % 否则，重新绘制小球
                surf(x * sphere_radius + center_x, ...
                     y * sphere_radius + center_y, ...
                     z * sphere_radius + center_z, ...
                     'EdgeColor', 'y', 'FaceColor', 'y');
            end
        end
    end
end

hold off;
