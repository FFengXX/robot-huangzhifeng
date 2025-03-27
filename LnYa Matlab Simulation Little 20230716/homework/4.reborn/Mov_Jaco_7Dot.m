close all;
clear;
global th1 th2 th3 d4 th5 th6 th7 garbage_points all_coordinates current_index corner_points corner_index rand_points;

% 初始化全局变量
th1=0;
th2=90;
th3=0;
d4=0;
th5=0;
th6=0;
th7=-90;
current_index = 1;
all_coordinates = zeros(3, 100000);  % 轨迹存储数组
corner_points = zeros(3, 8);        % 角点存储数组（固定8列）
corner_index = 1;                    % 角点索引计数器
rand_points = [];                   % 新增：小球坐标全局变量

garbage_points = [300, 400, 400];  % 行向量

% ================== 修改后的generate_workspace函数 ==================
function generate_workspace(corner_points)
    global rand_points;  % 声明全局变量
    
    % 计算工作空间范围
    x_lim = [min(corner_points(1,:)), max(corner_points(1,:))];
    y_lim = [min(corner_points(2,:)), max(corner_points(2,:))];
    z_lim = [min(corner_points(3,:)), max(corner_points(3,:))];
    
    % 生成7个随机黄色小球并存储到全局变量
    num_balls = 10;
    rand_points = [
        x_lim(1) + (x_lim(2)-x_lim(1))*rand(1,num_balls);
        y_lim(1) + (y_lim(2)-y_lim(1))*rand(1,num_balls);
        z_lim(1) + (z_lim(2)-z_lim(1))*rand(1,num_balls)
    ];
    
    % 获取当前figure1的句柄并保持绘图
    figure(1); 
    hold on;
    
    % 1. 绘制工作空间框架
    plot3(corner_points(1,[1:4 1]), corner_points(2,[1:4 1]), corner_points(3,[1:4 1]), 'b-'); 
    plot3(corner_points(1,[5:8 5]), corner_points(2,[5:8 5]), corner_points(3,[5:8 5]), 'b-');
    for i = 1:4
        plot3([corner_points(1,i), corner_points(1,i+4)],...
              [corner_points(2,i), corner_points(2,i+4)],...
              [corner_points(3,i), corner_points(3,i+4)], 'b-');
    end
    
    % 2. 绘制黄色小球
    plot3(rand_points(1,:), rand_points(2,:), rand_points(3,:),...
         'oy', 'MarkerSize', 12,...
         'MarkerFaceColor', 'y',...
         'MarkerEdgeColor', 'k',...
         'LineWidth', 1.5);
    
    axis equal; grid on;
    xlabel('x'); ylabel('y'); zlabel('z');
    title('Workspace with Random Balls');
end

function path = RRT(start, goal, obstacles, radius)
    max_iter = 50000;
    step_size = 100;
    goal_threshold = 50;
    bias = 0.1;
    global corner_points;
    
    % 获取工作空间范围
    x_lim = [min(corner_points(1,:)), max(corner_points(1,:))];
    y_lim = [min(corner_points(2,:)), max(corner_points(2,:))];
    z_lim = [min(corner_points(3,:)), max(corner_points(3,:))];
    
    % 初始化树
    nodes = struct('pos', {}, 'parent', {});
    nodes(1).pos = start;
    nodes(1).parent = 0;
    found = false;
    
    for iter = 1:max_iter
        % 生成随机点（带目标偏向）
        if rand < bias
            sample = goal;
        else
            sample = [
                x_lim(1) + (x_lim(2)-x_lim(1))*rand();
                y_lim(1) + (y_lim(2)-y_lim(1))*rand();
                z_lim(1) + (z_lim(2)-z_lim(1))*rand()
            ];
        end
        
        % 寻找最近节点
        [nearest_idx, min_dist] = find_nearest(nodes, sample);
        nearest_node = nodes(nearest_idx).pos;
        
        % 生成新节点
        direction = sample - nearest_node;
        distance = norm(direction);
        if distance > step_size
            new_node = nearest_node + direction/distance*step_size;
        else
            new_node = sample;
        end
        
        % 碰撞检测
        if ~collision_check(nearest_node, new_node, obstacles, radius)
            % 添加新节点
            new_idx = length(nodes)+1;
            nodes(new_idx).pos = new_node;
            nodes(new_idx).parent = nearest_idx;
            
            % 检查目标
            if norm(new_node - goal) < goal_threshold
                found = true;
                break;
            end
        end
    end
    
    if ~found
        error('Path not found');
    end
    
    % 回溯路径
    path = [];
    current_idx = new_idx;
    while current_idx ~= 0
        path = [nodes(current_idx).pos, path];
        current_idx = nodes(current_idx).parent;
    end
end

function [idx, min_dist] = find_nearest(nodes, sample)
    min_dist = inf;
    idx = 1;
    for i = 1:length(nodes)
        d = norm(nodes(i).pos - sample);
        if d < min_dist
            min_dist = d;
            idx = i;
        end
    end
end

function collision = collision_check(p1, p2, obstacles, r)
    collision = false;
    for i = 1:size(obstacles,2)
        % 计算线段到球心的最小距离
        v = p2 - p1;
        u = obstacles(:,i) - p1;
        t = dot(v,u)/dot(v,v);
        t = max(0, min(1,t));
        closest = p1 + t*v;
        dist = norm(closest - obstacles(:,i));
        
        if dist < r
            collision = true;
            return;
        end
    end
end


% ================== 修改后的MOVE_vector函数 ==================
function [all_xyz2, t] = MOVE_vector(times, print, len_x, len_y, len_z, record_corner, colour)
    global th1 th2 th3 d4 th5 th6 th7 all_coordinates garbage_points current_index corner_points corner_index rand_points;

    % 设置默认参数（新增colour参数）
    if nargin < 8
        colour = 'b';  % 默认颜色为蓝色
    end

    % 动态生成小球位置（如果未初始化）
    if isempty(rand_points)
        warning('rand_points未初始化，生成默认位置');
        rand_points = zeros(3,10);  % 生成7个原点位置
    end

    step_x = len_x/times;
    step_y = len_y/times;
    step_z = len_z/times;

    t = 1:times;
    all_xyz = zeros(3, times);
    
    % 主循环
    for i = 1:times
        cla(gca);  
        if print
            % 使用colour参数设置轨迹颜色
            plot3(all_coordinates(1,:), all_coordinates(2,:), all_coordinates(3,:),...
                 [colour '.'], 'MarkerSize', 5);  % 修改此行
            % 动态绘制当前小球位置
            plot3(rand_points(1,:), rand_points(2,:), rand_points(3,:),...
                 'oy', 'MarkerSize', 12,...
                 'MarkerFaceColor', 'y',...
                 'MarkerEdgeColor', 'k',...
                 'LineWidth', 1.5);
        end
        xyz = DHfk7Dof_Lnya2(th1, th2, th3, d4, th5, th6,th7, 0);
        all_xyz(:, i) = xyz;

        if print
            all_coordinates(:, current_index:current_index+times-1) = all_xyz;
        end

        J = Jacobian7DoF_Ln(th1, th2, th3, d4, th5, th6,th7); 
        
        dD = [step_x, step_y, step_z, 0, 0, 0]';
        dth = pinv(J) * dD;
        
        th1 = th1 + dth(1) * 180/pi;
        th2 = th2 + dth(2) * 180/pi;
        th3 = th3 + dth(3) * 180/pi;
        d4  = d4  + dth(4) * 180/pi;
        th5 = th5 + dth(5) * 180/pi;
        th6 = th6 + dth(6) * 180/pi;
        th7 = th7 + dth(7) * 180/pi;
        drawnow limitrate;
    end
    
    % ========== 角点存储逻辑 ==========
    if record_corner  % 使用新参数控制角点记录
        if corner_index > size(corner_points, 2)
            error('角点数量超过预分配空间，请扩大corner_points数组');
        end
        corner_points(:, corner_index) = xyz;
        corner_index = corner_index + 1;
        
        fprintf('[角点记录] 新增点%d: (%.1f, %.1f, %.1f)\n',...
                corner_index-1, xyz(1), xyz(2), xyz(3));
    end
    
    if(print)
        fprintf('x= %f, y= %f, z= %f\n', xyz(1), xyz(2), xyz(3));
    end
    
    if print
        current_index = current_index + times;
    end
    all_xyz2 = all_xyz;
end
% ================== 修改后的workspace函数 ==================
function workspace()
    global corner_points rand_points;
    
    % 初始化位置（不记录轨迹和角点）
    MOVE_vector(100, false, 300, 300, 0, false); 
    
    % 执行8次边界运动（记录轨迹和角点）
    MOVE_vector(100, true, 0, 1000, 0, true,'b');
    MOVE_vector(100, true, 0, 0, -2000, true,'b');
    MOVE_vector(100, true, 0, -2000, 0, true,'b');
    MOVE_vector(100, true, 0, 0, 2000, true,'b');
    MOVE_vector(100, true, -1000, 0, 0, true,'b');
    MOVE_vector(100, true, 0, 2000, 0, true,'b');
    MOVE_vector(100, true, 0, 0, -2000, true,'b');
    MOVE_vector(100, true, 0, -2000, 0, true,'b');

    % 生成工作空间
    generate_workspace(corner_points);
    
    % 打印角点坐标
    fprintf('\n======= 工作空间角点坐标 =======\n');
    for i = 1:size(corner_points,2)
        fprintf('角点%d: x=%.1f, y=%.1f, z=%.1f\n',...
                i, corner_points(1,i), corner_points(2,i), corner_points(3,i));
    end
end


% 修改后的move_to_target函数
function move_to_target(target_x, target_y, target_z)
    global th1 th2 th3 d4 th5 th6 th7 rand_points;
    
    % 获取当前坐标
    current_xyz = DHfk7Dof_Lnya2(th1, th2, th3, d4, th5, th6, th7, 0);
    
    % RRT参数设置
    start = current_xyz;
    goal = [target_x; target_y; target_z];
    obstacles = rand_points;
    radius = 30;  % 小球碰撞半径
    
    % 路径规划
    try
        path = RRT(start, goal, obstacles, radius);
    catch
        error('路径规划失败');
    end
    
    % 沿路径移动
    for i = 1:size(path,2)-1
        delta = path(:,i+1) - path(:,i);
        MOVE_vector(50, true, delta(1), delta(2), delta(3), false, 'g');
    end
    
    % 移除已抓取的小球
    dist = sum((rand_points - goal).^2);
    [~, idx] = min(dist);
    rand_points(:,idx) = [];
end
function sort_rand_points_by_distance()
    global rand_points garbage_points;
    
    % 计算每个小球到垃圾桶的欧氏距离
    distances = sqrt(...
        (rand_points(1,:) - garbage_points(1)).^2 + ...
        (rand_points(2,:) - garbage_points(2)).^2 + ...
        (rand_points(3,:) - garbage_points(3)).^2 ...
    );
    
    % 按距离从小到大排序，获取排序后的索引
    [~, sorted_indices] = sort(distances);
    
    % 按排序后的索引重新排列小球坐标
    rand_points = rand_points(:, sorted_indices);
    
    % 打印排序结果（可选）
    fprintf('\n======= 按距离排序后的小球坐标 =======\n');
    for i = 1:size(rand_points,2)
        fprintf('小球%d: 距离=%.2fmm, 坐标=(%.1f, %.1f, %.1f)\n',...
                i, distances(sorted_indices(i)),...
                rand_points(1,i), rand_points(2,i), rand_points(3,i));
    end
end
% ================== 主程序 ==================
workspace();
sort_rand_points_by_distance();  % 按距离排序小球

for i = 1:size(rand_points,2)
    move_to_target(rand_points(1,i), rand_points(2,i), rand_points(3,i));
end
move_to_target(garbage_points(1),garbage_points(2),garbage_points(3));