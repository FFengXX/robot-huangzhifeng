close all;
clear;
global th1 th2 th3 th4 th5 th6 all_coordinates current_index;

% 初始化全局变量
th1 = 0;
th2 = 90;
th3 = 0;
th4 = 0;
th5 = 0;
th6 = 0;
current_index = 1;  % 当前存储位置索引
%all_coordinates = zeros(3, 400);  % 预分配总空间（4次×100点）
all_coordinates = zeros(3, 100000);  % 预分配总空间（4次×100点）
% 首次绘制初始状态
xyz = DHfk6Dof_Lnya(th1, th2, th3, th4, th5, th6, 0);

function [all_xyz2, xout, t] = MOVE_vector(times,print, len_x, len_y, len_z)
    global th1 th2 th3 th4 th5 th6 all_coordinates current_index;
    
    step_x = len_x/times;
    step_y = len_y/times;
    step_z = len_z/times;


    % 预分配内存
    xout = zeros(1, times);
    t = 1:times;
    all_xyz = zeros(3, times);
    
    % 主循环
    for i = 1:times
        cla(gca);  
        if print == true
            plot3(all_coordinates(1,:), all_coordinates(2,:), all_coordinates(3,:), 'b.', 'MarkerSize', 10);
        end

        % 计算当前状态
        xyz = DHfk6Dof_Lnya(th1, th2, th3, th4, th5, th6, 0);
        all_xyz(:, i) = xyz;
        
        % 更新全局轨迹数组
        if current_index + times - 1 > size(all_coordinates, 2)
            error('预分配空间不足，请扩大 all_coordinates 的列数');
        end

        if print ~=false
            all_coordinates(:, current_index:current_index+times-1) = all_xyz;
        end

        % 计算雅可比矩阵
        J = Jacobian6DoF_Ln(th1, th2, th3, th4, th5, th6); 
        xout(i) = det(J);
        
        % 逆运动学更新
        dD = [step_x, step_y, step_z, 0, 0, 0]';
        dth = pinv(J) * dD;  % 改用伪逆提高稳定性
        
        th1 = th1 + dth(1) * 180/pi;
        th2 = th2 + dth(2) * 180/pi;
        th3 = th3 + dth(3) * 180/pi;
        th4 = th4 + dth(4) * 180/pi;
        th5 = th5 + dth(5) * 180/pi;
        th6 = th6 + dth(6) * 180/pi;
        
        drawnow limitrate;
end

    if(print)
        fprintf('x= %f, y= %f, z= %f\n', xyz(1), xyz(2), xyz(3));
    end
    % 更新存储索引
    current_index = current_index + times;
    all_xyz2 = all_xyz;  % 返回本次轨迹数据（可选）
end

%MOVE_vector(100,3000,3000,3000);
%MOVE_vector(100,-6000,-6000,-6000);



% 调用MOVE函数（自动更新全局变量）
%初始位置调整
function workspace()
MOVE_vector(100,false, 4000, 0, 0); 

MOVE_vector(100,true, 0, 10000, 0);  
MOVE_vector(100,true, 0, 0, -20000);  
MOVE_vector(100,true, 0, -20000, 0);  
MOVE_vector(100,true, 0, 0, 20000); 

MOVE_vector(100,true, -10000, 0, 0); 

MOVE_vector(100,true, 0, 20000, 0);  
MOVE_vector(100,true, 0, 0, -20000);  
MOVE_vector(100,true, 0, -20000, 0);  
end
workspace();


% 绘制全局轨迹
figure;
plot3(all_coordinates(1,:), all_coordinates(2,:), all_coordinates(3,:),...
      'r.', 'MarkerSize', 10);
axis equal;
grid on;
xlabel('x'); ylabel('y'); zlabel('z');
title('累积轨迹');