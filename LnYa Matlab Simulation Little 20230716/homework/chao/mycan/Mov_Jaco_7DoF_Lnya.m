
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 1.0  正常运行
% % 无误差分析
% close all;
% clear;
% 
% 
% ToDeg = 180/pi;
% ToRad = pi/180;
% 
% 
% th1=0;
% th2=90;
% th3=0;
% th4=0;
% d5=0;
% th6=0;
% th7=0;
% DHfk7Dof_Lnya(th1,th2,th3,th4,d5,th6,th7,0);
% view(134,12);
% pause;
% stp=15;
% %% 微分运动
% pause;
% cla;
% th1=0;
% th2=90;
% th3=0;
% th4=0;
% d5=0;
% th6=0;
% th7=0;
% cla;
% 
% for  i=1:20
%     DHfk7Dof_Lnya(th1,th2,th3,th4,d5,th6,th7,0);
%     J=Jacobian7DoF_Ln(th1,th2,th3,th4,d5,th6,th7);
%     dD=[0 0 1 0 0 0]';
%     dth=pinv(J)*dD;
%     th1=th1+dth(1)/pi*180;
%     th2=th2+dth(2)/pi*180;
% %     disp(th2);
% %     disp(dth(2)/pi*180);
%     th3=th3+dth(3)/pi*180;
%     th4=th4+dth(4)/pi*180;
%     d5=d5+dth(5); 
%     th6=th6+dth(6)/pi*180;
%     th7=th7+dth(7)/pi*180;
% end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%有分析
close all;
clear;
clc;

% 常量定义
ToDeg = 180/pi;
ToRad = pi/180;

% 初始化关节角度
th1 = 0;
th2 = 90;
th3 = 0;
th4 = 0;
d5 = 0;
th6 = 0;
th7 = 0;

% 计算机械臂初始位置
[initial_x, initial_y, initial_z] = DHfk7Dof_Lnya(th1, th2, th3, th4, d5, th6, th7, 1);

% 初始化误差记录数组
error_history = [];

% 绘制初始状态
DHfk7Dof_Lnya(th1, th2, th3, th4, d5, th6, th7, 0);
view(134, 12);
pause;

% 微分运动验证
cla;
for i = 1:20
   
    DHfk7Dof_Lnya(th1, th2, th3, th4, d5, th6, th7, 0);
    hold on;
    
 
    J = Jacobian7DoF_Ln(th1, th2, th3, th4, d5, th6, th7);
    
    dD = [0; 0; 5; 0; 0; 0]; 
    
    lambda = 0.01; % 通常取 0.01 或更小的值
 dth =J' / (J * J' + lambda^2 * eye(6)) * dD ;
%     dth=pinv(J)*dD;
    
    % 更新关节角度
    th1 = th1 + dth(1) / pi * 180;
    th2 = th2 + dth(2) / pi * 180;
    th3 = th3 + dth(3) / pi * 180;
    th4 = th4 + dth(4) / pi * 180;
    d5 = d5 + dth(5); % 伸缩关节，直接加上长度变化量
    th6 = th6 + dth(6) / pi * 180;
    th7 = th7 + dth(7) / pi * 180;
    
    % 计算机械臂末端执行器的实际位置
    [ex, ey, ez] = DHfk7Dof_Lnya(th1, th2, th3, th4, d5, th6, th7, 1);
    
    % 计算理想位置（在初始位置的 z 轴上每次加 1）
    ideal_z = initial_z + i * 1; % 初始 z 坐标 + i * 1
    ideal_position = [initial_x; initial_y; ideal_z]; % 理想位置 [initial_x, initial_y, initial_z + i*1]
    
    % 计算误差（理想位置 - 实际位置）
    p_err = norm(ideal_position - [ex; ey; ez]);
    
    % 保存误差到数组
    error_history = [error_history; p_err];
    
    % 打印调试信息
    fprintf('迭代 %d:\n', i);
    fprintf('理想位置: [%.4f, %.4f, %.4f]\n', initial_x, initial_y, ideal_z);
    fprintf('实际位置: [%.4f, %.4f, %.4f]\n', ex, ey, ez);
    fprintf('误差: %.6f\n', p_err);
    
    % 绘制末端执行器的轨迹
    plot3(ex, ey, ez, 'r.');
    drawnow;
    
    % 暂停以观察运动
    pause(0.1);
end
hold off;

% 绘制误差变化曲线
figure;
plot(1:length(error_history), error_history, '-o', 'LineWidth', 1.5);
xlabel('迭代次数');
ylabel('误差');
title('误差变化曲线');
grid on;
