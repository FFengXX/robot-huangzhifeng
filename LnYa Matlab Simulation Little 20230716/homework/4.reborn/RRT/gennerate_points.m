close all;
clear;
global th1 th2 th3 d4 th5 th6 th7 garbage_points all_coordinates current_index corner_points corner_index rand_points captured_joint_angles;

% ================== 初始化全局变量 ==================
th1=0;
th2=90;
th3=0;
d4=0;
th5=0;
th6=0;
th7=-90;
current_index = 1;
all_coordinates = zeros(3, 100000);
corner_points = [1532, 532, 532, 1532, 1532, 532, 532, 1532;
                 677, 677, -1323, -1323, 677, 677, -1323, -1323;
                 2139, 2139, 2139, 2139, 139, 139, 139, 139];
corner_index = 1;
rand_points = [];   
captured_joint_angles = [];  
garbage_points = [300, 400, 400];

% ================== 清空数据文件 ==================
fclose(fopen('障碍物.txt', 'w'));
fclose(fopen('收敛角度.txt', 'w'));

% ================== 工作空间生成函数 ==================
function generate_workspace(corner_points)
    global rand_points;
    x_lim = [min(corner_points(1,:)), max(corner_points(1,:))];
    y_lim = [min(corner_points(2,:)), max(corner_points(2,:))];
    z_lim = [min(corner_points(3,:)), max(corner_points(3,:))];
    
    num_balls = 10;
    rand_points = [
        x_lim(1) + (x_lim(2)-x_lim(1))*rand(1,num_balls);
        y_lim(1) + (y_lim(2)-y_lim(1))*rand(1,num_balls);
        z_lim(1) + (z_lim(2)-z_lim(1))*rand(1,num_balls)
    ];
    
    figure(1); 
    hold on;
    plot3(corner_points(1,[1:4 1]), corner_points(2,[1:4 1]), corner_points(3,[1:4 1]), 'b-'); 
    plot3(corner_points(1,[5:8 5]), corner_points(2,[5:8 5]), corner_points(3,[5:8 5]), 'b-');
    for i = 1:4
        plot3([corner_points(1,i), corner_points(1,i+4)],...
              [corner_points(2,i), corner_points(2,i+4)],...
              [corner_points(3,i), corner_points(3,i+4)], 'b-');
    end
    plot3(rand_points(1,:), rand_points(2,:), rand_points(3,:),...
         'oy', 'MarkerSize', 12, 'MarkerFaceColor', 'y', 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
    axis equal; grid on;
    xlabel('x'); ylabel('y'); zlabel('z');
    title('Workspace with Random Balls');
end

% ================== 运动控制函数 ==================
function [all_xyz2, t] = MOVE_vector(times, print, len_x, len_y, len_z, colour)
    global th1 th2 th3 d4 th5 th6 th7 all_coordinates current_index rand_points corner_points;
    if nargin < 6, colour = 'b'; end
    
    step_x = len_x/times;
    step_y = len_y/times;
    step_z = len_z/times;
    t = 1:times;
    all_xyz = zeros(3, times);
    
    for i = 1:times
        cla(gca);  
        if print
            plot3(all_coordinates(1,:), all_coordinates(2,:), all_coordinates(3,:),...
                 [colour '.'], 'MarkerSize', 5);
            plot3(corner_points(1,[1:4 1]), corner_points(2,[1:4 1]), corner_points(3,[1:4 1]), 'b-'); 
            plot3(corner_points(1,[5:8 5]), corner_points(2,[5:8 5]), corner_points(3,[5:8 5]), 'b-');
            for j = 1:4
                plot3([corner_points(1,j), corner_points(1,j+4)],...
                      [corner_points(2,j), corner_points(2,j+4)],...
                      [corner_points(3,j), corner_points(3,j+4)], 'b-');
            end
            plot3(rand_points(1,:), rand_points(2,:), rand_points(3,:),...
                 'oy', 'MarkerSize', 12, 'MarkerFaceColor', 'y', 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
        end
        
        xyz = DHfk7Dof_Lnya2(th1, th2, th3, d4, th5, th6, th7, 0);
        all_xyz(:, i) = xyz;

        if print
            all_coordinates(:, current_index:current_index+times-1) = all_xyz;
        end

        J = Jacobian7DoF_Ln(th1, th2, th3, d4, th5, th6, th7); 
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
    
    if(print)
        fprintf('x= %f, y= %f, z= %f\n', xyz(1), xyz(2), xyz(3));
        current_index = current_index + times;
    end
    all_xyz2 = all_xyz;
end

% ================== 目标移动函数 ==================
function move_to_target(target_x, target_y, target_z)
    global th1 th2 th3 d4 th5 th6 th7 rand_points captured_joint_angles corner_points;
    
    current_xyz = DHfk7Dof_Lnya2(th1, th2, th3, d4, th5, th6, th7, 0);
    dx = target_x - current_xyz(1);
    dy = target_y - current_xyz(2);
    dz = target_z - current_xyz(3);
    
    MOVE_vector(10, true, dx, dy, dz, 'g');
    final_xyz = DHfk7Dof_Lnya2(th1, th2, th3, d4, th5, th6, th7, 0);
    
    % 保存误差数据
    fprintf('\n目标点: (%.1f, %.1f, %.1f)\n实际到达: (%.1f, %.1f, %.1f)\n误差: %.2fmm\n',...
            target_x, target_y, target_z, final_xyz(1), final_xyz(2), final_xyz(3),...
            norm(final_xyz - [target_x; target_y; target_z]));
    
    collision_threshold = 20;
    distances = sqrt( (rand_points(1,:) - final_xyz(1)).^2 + ...
                      (rand_points(2,:) - final_xyz(2)).^2 + ...
                      (rand_points(3,:) - final_xyz(3)).^2 );
    collided_idx = find(distances < collision_threshold);
    
    if ~isempty(collided_idx)
        % 保存关节角度和坐标数据
        fprintf('抓到小球\n');
        fprintf('当前关节角度: th1=%.2f, th2=%.2f, th3=%.2f, d4=%.2f, th5=%.2f, th6=%.2f, th7=%.2f\n',...
                th1, th2, th3, d4, th5, th6, th7);
        
        % 保存到文件
        dlmwrite('障碍物.txt', final_xyz', '-append', 'delimiter', ' ', 'precision', '%.1f');
        dlmwrite('收敛角度.txt', [th1, th2, th3, d4, th5, th6, th7], '-append', 'delimiter', ' ', 'precision', '%.2f');
        
        % 更新工作空间
        rand_points(:, collided_idx(1)) = [];
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
              'oy', 'MarkerSize', 12, 'MarkerFaceColor', 'y', 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
        hold off;
    end
end

% ================== 距离排序函数 ==================
function sort_rand_points_by_distance()
    global rand_points garbage_points;
    distances = sqrt(...
        (rand_points(1,:) - garbage_points(1)).^2 + ...
        (rand_points(2,:) - garbage_points(2)).^2 + ...
        (rand_points(3,:) - garbage_points(3)).^2 ...
    );
    [~, sorted_indices] = sort(distances);
    rand_points = rand_points(:, sorted_indices);
    
    fprintf('\n======= 按距离排序后的小球坐标 =======\n');
    for i = 1:size(rand_points,2)
        fprintf('小球%d: 距离=%.2fmm, 坐标=(%.1f, %.1f, %.1f)\n',...
                i, distances(sorted_indices(i)),...
                rand_points(1,i), rand_points(2,i), rand_points(3,i));
    end
end

% ================== 主程序 ==================
generate_workspace(corner_points);
sort_rand_points_by_distance();

while ~isempty(rand_points)
    move_to_target(rand_points(1,1), rand_points(2,1), rand_points(3,1));
end