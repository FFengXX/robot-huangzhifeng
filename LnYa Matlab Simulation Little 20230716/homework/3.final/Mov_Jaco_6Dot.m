close all;
clear;
% 通过正运动学分析当前状态，并通过逆运动学迭代调整关节角度，实现末端执行器的定向运动
% 初始的参数只有th1-th6

% 这个脚本循环做了三件事
% 正运动学计算：
% 根据当前关节角度绘制机械臂，并得到末端位置。

% 逆运动学求解：
% 计算雅可比矩阵 J，然后通过 雅可比逆矩阵 将期望的末端位移 dD = [20, 0, 0, 0, 0, 0]'（沿 X 轴移动 20 单位）转换为关节角度增量 dth：

% 更新关节角度：
% 将 dth 转换为角度并叠加到当前关节角度，使末端逐步向目标方向移动。


global th1 th2 th3 th4 th5 th6;
th1 = 0;
th2 = 90;
th3 = 0;
th4 = 0;
th5 = 0;
th6 = 0;

% 预分配总存储数组（假设最多运行3次，每次100点）
all_coordinates = zeros(3, 300);  % 3行×300列
current_index = 1;  % 当前存储位置索引

% 首次绘制初始状态
xyz = DHfk6Dof_Lnya(th1, th2, th3, th4, th5, th6, 0);






function [all_xyz2, xout,t] = MOVE(times,step_x,step_y,step_z)
global th1 th2 th3 th4 th5 th6;
% 预分配内存加速循环
num = 1;
xout = zeros(1, 1000);
t = zeros(1, 1000);

% 禁用坐标轴自动缩放
ax = gca;
ax.XLimMode = 'manual';
ax.YLimMode = 'manual';
ax.ZLimMode = 'manual';
axis([-3000, 3000, -3000, 3000, -3000, 3000]);  % 固定坐标范围
all_xyz = zeros(3, times);

% 主循环
    for i = 1:times
        % 清除当前图形内容（仅清空子图，保留窗口）
        cla(ax);  
        
        % 绘制当前机械臂状态
    
        xyz =DHfk6Dof_Lnya(th1, th2, th3, th4, th5, th6, 0);
        all_xyz(:, i) = [xyz(1); xyz(2); xyz(3)];
        fprintf('x= %2.4f y= %2.4f z= %2.4f \n', xyz(1),xyz(2),xyz(3));
        
        % 计算雅可比矩阵和行列式
        J = Jacobian6DoF_Ln(th1, th2, th3, th4, th5, th6); 
        x = det(J);
        fprintf('奇异点x= %2.4f \n', x);  
        xout(num) = x;
        t(num) = i;
        num = num + 1;
        
        % 逆运动学求解
        dD = [step_x step_y step_z 0 0 0]';
        dth = inv(J) * dD;
        
        % 更新关节角度（优化计算顺序）
        th1 = th1 + dth(1) * 180/pi;  % 正确弧度转角度公式
        th2 = th2 + dth(2) * 180/pi;
        th3 = th3 + dth(3) * 180/pi;
        th4 = th4 + dth(4) * 180/pi;
        th5 = th5 + dth(5) * 180/pi;
        th6 = th6 + dth(6) * 180/pi;
        
        % 强制刷新图形（禁用默认双缓冲）
        drawnow limitrate;  
    end
    all_xyz2 = all_xyz;
end





[coordinate1, xout1, t1] = MOVE(100, -200, 0, 0);
all_coordinates(:, current_index:current_index+99) = coordinate1;
current_index = current_index + 100;

[coordinate2, xout2, t2] = MOVE(100, 0, 0, -200);
all_coordinates(:, current_index:current_index+99) = coordinate2;
current_index = current_index + 100;

[coordinate3, xout3, t3] = MOVE(100, 200, 0, 0);
all_coordinates(:, current_index:current_index+99) = coordinate3;
current_index = current_index + 100;


[coordinate4, xout3, t3] = MOVE(100, 0, 0, 200);
all_coordinates(:, current_index:current_index+99) = coordinate4;
current_index = current_index + 100;





% 统一绘制所有轨迹点
%figure;
%plot3(all_coordinates(1,:), all_coordinates(2,:), all_coordinates(3,:), 'r.', 'MarkerSize', 10);
%axis equal;
%grid on;
%xlabel('x'); ylabel('y'); zlabel('z');
%title('累积轨迹');

% 绘制行列式变化曲线（独立窗口）
%figure('Name', 'Jacobian Determinant');
%plot(t, xout, 'r-O');
%axis tight;
%grid on;
%xlabel('Iteration');
%ylabel('det(J)');
