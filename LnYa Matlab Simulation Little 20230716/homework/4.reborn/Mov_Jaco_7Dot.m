close all;
clear;
global th1 th2 th3 d4 th5 th6 th7 garbage_points current_index corner_points corner_index rand_points captured_joint_angles;
% 初始化全局变量
th1=0;
th2=90;
th3=0;
d4=0;
th5=0;
th6=0;
th7=-90;
start_q = [0, 90, 0, 0, 0, 0, -90];  

current_index = 1;

corner_points = [1532, 532, 532, 1532, 1532, 532, 532, 1532;
                 677, 677, -1323, -1323, 677, 677, -1323, -1323;
                 2139, 2139, 2139, 2139, 139, 139, 139, 139];
% 小球的笛卡尔坐标系坐标数组
rand_points =     [818.0, 714.1,  1077.0, 899.5,  1433.4, 1268.5, 1449.7, 1344.9, 695.3,  1263.0;
                   136.2, -243.6, -102.2, -226.2, -15.1,  211.0,  -738.5, -715.6, -770.5, 551.0;
                   895.5, 537.3,  1121.8, 1302.8, 1217.8, 1635.9, 535.4,  1298.6, 1725.7, 2013.2];
% 小球的关节角坐标数组
ball_joint_positions = [
    -38.81, 104.15, 45.54, 0.55, -153.36, -41.78, 22.99;  % 小球1
    -14.73, 130.86, 38.85, -12.04, -202.16, -31.98, 80.42; % 小球2
    -35.44, 82.80, -23.03, 24.23, -94.80, -29.92, 32.26;   % 小球3
    -52.11, 75.30, -27.64, 26.53, -73.74, -40.99, 14.27;   % 小球4
    -31.74, 65.95, -29.94, 29.90, -71.02, -17.63, 8.79;    % 小球5
    -24.29, 84.17, -36.90, 7.19, -53.69, -20.08, 0.62;     % 小球6
    -31.08, -14.02, -24.03, 78.23, -17.54, -24.52, -79.42; % 小球7
    -58.40, 31.66, -53.29, 54.36, -24.84, -20.48, -41.35;  % 小球8
    -95.67, 52.90, -97.42, 47.40, 5.36, -46.31, -41.04;    % 小球9
    -49.23, 89.20, -90.63, 44.21, 7.35, -4.32, -7.33       % 小球10
];

corner_index = 1;   % 角点索引计数器
captured_joint_angles = [];  % 用于存储抓取时的关节角度
garbage_points = [532, -1323, 2139];  % 垃圾桶坐标


% ================== 修改后的generate_workspace函数 ==================
function generate_workspace(corner_points)
    global rand_points;  % 声明全局变量
    
    % 固定的小球坐标（每列为一个小球的[x; y; z]）
    rand_points = [818.0, 714.1, 1077.0, 899.5, 1433.4, 1268.5, 1449.7, 1344.9, 695.3, 1263.0;
                   136.2, -243.6, -102.2, -226.2, -15.1, 211.0, -738.5, -715.6, -770.5, 551.0;
                   895.5, 537.3, 1121.8, 1302.8, 1217.8, 1635.9, 535.4, 1298.6, 1725.7, 2013.2];
    
    % 获取当前figure1的句柄并保持绘图
    figure(1); 
    hold on;
    
    % 绘制工作空间框架
    plot3(corner_points(1,[1:4 1]), corner_points(2,[1:4 1]), corner_points(3,[1:4 1]), 'b-'); 
    plot3(corner_points(1,[5:8 5]), corner_points(2,[5:8 5]), corner_points(3,[5:8 5]), 'b-');
    for i = 1:4
        plot3([corner_points(1,i), corner_points(1,i+4)],...
              [corner_points(2,i), corner_points(2,i+4)],...
              [corner_points(3,i), corner_points(3,i+4)], 'b-');
    end
    
    % 绘制固定的小球
    plot3(rand_points(1,:), rand_points(2,:), rand_points(3,:),...
         'oy', 'MarkerSize', 12,...
         'MarkerFaceColor', 'y',...
         'MarkerEdgeColor', 'k',...
         'LineWidth', 1.5);
    
    axis equal; grid on;
    xlabel('x'); ylabel('y'); zlabel('z');
    title('Workspace with Fixed Balls');
end


% ================== 修改后的MOVE_vector函数 ==================
function [all_xyz2, t] = MOVE_vector(times, print, len_x, len_y, len_z, colour)
    global th1 th2 th3 d4 th5 th6 th7 all_coordinates  current_index rand_points corner_points;

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

            %  绘制工作空间框架
            plot3(corner_points(1,[1:4 1]), corner_points(2,[1:4 1]), corner_points(3,[1:4 1]), 'b-'); 
            plot3(corner_points(1,[5:8 5]), corner_points(2,[5:8 5]), corner_points(3,[5:8 5]), 'b-');
            for i = 1:4
                plot3([corner_points(1,i), corner_points(1,i+4)],...
                      [corner_points(2,i), corner_points(2,i+4)],...
                      [corner_points(3,i), corner_points(3,i+4)], 'b-');
            end

            % 动态绘制当前小球位置
            plot3(rand_points(1,:), rand_points(2,:), rand_points(3,:),...
                 'oy', 'MarkerSize', 12,...
                 'MarkerFaceColor', 'y',...
                 'MarkerEdgeColor', 'k',...
                 'LineWidth', 1.5);
        end
        xyz = DHfk7Dof_Lnya2(th1, th2, th3, d4, th5, th6,th7, 0);
        all_xyz(:, i) = xyz;


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
    % if record_corner  % 使用新参数控制角点记录
    %     if corner_index > size(corner_points, 2)
    %         error('角点数量超过预分配空间，请扩大corner_points数组');
    %     end
    %     corner_points(:, corner_index) = xyz;
    %     corner_index = corner_index + 1;
    % 
    %     fprintf('[角点记录] 新增点%d: (%.1f, %.1f, %.1f)\n',...
    %             corner_index-1, xyz(1), xyz(2), xyz(3));
    % end
    % 
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
    global corner_points;
    % 生成工作空间
    generate_workspace(corner_points);

end
function move_to_target(target_x, target_y, target_z)
    global th1 th2 th3 d4 th5 th6 th7 rand_points captured_joint_angles corner_points;
    
    % 获取当前坐标
    current_xyz = DHfk7Dof_Lnya2(th1, th2, th3, d4, th5, th6, th7, 0);
    
    % 计算位移
    dx = target_x - current_xyz(1);
    dy = target_y - current_xyz(2);
    dz = target_z - current_xyz(3);
    
    % 执行运动（不记录轨迹和角点）
    MOVE_vector(100, true, dx, dy, dz, 'g');
    
    % 验证最终位置
    final_xyz = DHfk7Dof_Lnya2(th1, th2, th3, d4, th5, th6, th7, 0);
    fprintf('\n目标点: (%.1f, %.1f, %.1f)\n实际到达: (%.1f, %.1f, %.1f)\n误差: %.2fmm\n',...
            target_x, target_y, target_z, final_xyz(1), final_xyz(2), final_xyz(3),...
            norm(final_xyz - [target_x; target_y; target_z]));
    
    collision_threshold = 20;  % 碰撞检测阈值，根据需要调整
    distances = sqrt( (rand_points(1,:) - final_xyz(1)).^2 + ...
                      (rand_points(2,:) - final_xyz(2)).^2 + ...
                      (rand_points(3,:) - final_xyz(3)).^2 );
    collided_idx = find(distances < collision_threshold);
    
    if ~isempty(collided_idx)
        fprintf('抓到小球\n');
        % 打印当前机械臂的关节角度
        fprintf('当前关节角度: th1=%.2f, th2=%.2f, th3=%.2f, d4=%.2f, th5=%.2f, th6=%.2f, th7=%.2f\n',...
                th1, th2, th3, d4, th5, th6, th7);
        % 将当前关节角度保存到全局变量（每列一个记录）
        captured_joint_angles = [captured_joint_angles, [th1; th2; th3; d4; th5; th6; th7]];
        
        % 从 rand_points 中删除该小球
        rand_points(:, collided_idx(1)) = [];
        
        % 更新绘图：清除当前图后重新绘制工作空间和剩余小球
        cla(gca);
        plot3(corner_points(1,[1:4 1]), corner_points(2,[1:4 1]), corner_points(3,[1:4 1]), 'b-'); 
        plot3(corner_points(1,[5:8 5]), corner_points(2,[5:8 5]), corner_points(3,[5:8 5]), 'b-');
        for i = 1:4
            plot3([corner_points(1,i), corner_points(1,i+4)],...
                  [corner_points(2,i), corner_points(2,i+4)],...
                  [corner_points(3,i), corner_points(3,i+4)], 'b-');
        end
        hold on;
        plot3(rand_points(1,:), rand_points(2,:), rand_points(3,:),...
              'oy', 'MarkerSize', 12, 'MarkerFaceColor', 'y',...
              'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
        hold off;
    end
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

while ~isempty(rand_points)
    % 始终取第一个小球作为目标
    move_to_target(rand_points(1,1), rand_points(2,1), rand_points(3,1));
end
