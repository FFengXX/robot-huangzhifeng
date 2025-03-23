close all;
clear;
global th1 th2 th3 th4 th5 th6 all_coordinates current_index corner_points corner_index rand_points;

% 初始化全局变量
th1 = 0;
th2 = 90;
th3 = 0;
th4 = 0;
th5 = 0;
th6 = 0;
current_index = 1;
all_coordinates = zeros(3, 100000);  % 轨迹存储数组
corner_points = zeros(3, 8);        % 角点存储数组（固定8列）
corner_index = 1;                    % 角点索引计数器
rand_points = [];                   % 新增：小球坐标全局变量

% ================== 修改后的generate_workspace函数 ==================
function generate_workspace(corner_points)
    global rand_points;  % 声明全局变量
    
    % 计算工作空间范围
    x_lim = [min(corner_points(1,:)), max(corner_points(1,:))];
    y_lim = [min(corner_points(2,:)), max(corner_points(2,:))];
    z_lim = [min(corner_points(3,:)), max(corner_points(3,:))];
    
    % 生成7个随机黄色小球并存储到全局变量
    num_balls = 7;
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

% ================== 修改后的MOVE_vector函数 ==================
function [all_xyz2, xout, t] = MOVE_vector(times, print, len_x, len_y, len_z)
    global th1 th2 th3 th4 th5 th6 all_coordinates current_index corner_points corner_index;
    
    step_x = len_x/times;
    step_y = len_y/times;
    step_z = len_z/times;

    xout = zeros(1, times);
    t = 1:times;
    all_xyz = zeros(3, times);
    
    % 主循环
    for i = 1:times
        cla(gca);  
        if print
            plot3(all_coordinates(1,:), all_coordinates(2,:), all_coordinates(3,:), 'b.');
        end
        
        xyz = DHfk6Dof_Lnya(th1, th2, th3, th4, th5, th6, 0);
        all_xyz(:, i) = xyz;
        
        if print
            all_coordinates(:, current_index:current_index+times-1) = all_xyz;
        end

        J = Jacobian6DoF_Ln(th1, th2, th3, th4, th5, th6); 
        xout(i) = det(J);
        
        dD = [step_x, step_y, step_z, 0, 0, 0]';
        dth = pinv(J) * dD;
        
        th1 = th1 + dth(1) * 180/pi;
        th2 = th2 + dth(2) * 180/pi;
        th3 = th3 + dth(3) * 180/pi;
        th4 = th4 + dth(4) * 180/pi;
        th5 = th5 + dth(5) * 180/pi;
        th6 = th6 + dth(6) * 180/pi;
        
        drawnow limitrate;
    end
    
    % ========== 角点存储逻辑 ==========
    if print
        if corner_index > size(corner_points, 2)
            error('角点数量超过预分配空间，请扩大corner_points数组');
        end
        corner_points(:, corner_index) = xyz;
        corner_index = corner_index + 1;
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
    
    % 初始化位置（不记录角点）
    MOVE_vector(100, false, 3000, 0, 0); 
    
    % 执行8次轨迹运动（记录角点）
    MOVE_vector(100, true, 0, 10000, 0);
    MOVE_vector(100, true, 0, 0, -20000);
    MOVE_vector(100, true, 0, -20000, 0);
    MOVE_vector(100, true, 0, 0, 20000);
    MOVE_vector(100, true, -10000, 0, 0);
    MOVE_vector(100, true, 0, 20000, 0);
    MOVE_vector(100, true, 0, 0, -20000);
    MOVE_vector(100, true, 0, -20000, 0);

    % 生成工作空间（自动保存小球坐标到rand_points）
    generate_workspace(corner_points);
    
    % 打印全局存储的小球坐标
    fprintf('\n======= 随机小球坐标 =======\n');
    for i = 1:size(rand_points,2)
        fprintf('小球 %d: x=%.2f, y=%.2f, z=%.2f\n',...
                i, rand_points(1,i), rand_points(2,i), rand_points(3,i));
    end
    fprintf('============================\n');
end

% ================== 主程序 ==================
workspace();