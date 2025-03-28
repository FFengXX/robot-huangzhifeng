clc;
clear all;
close all;

global Link 
num=1;
learning_rate = 2;
lambda = 0.01;
title('数值解求解--学习率:2--正则化参数:0.01');
rotate3d on;
hold on;

ToDeg = 180/pi;
ToRad = pi/180;
count = 0;

%输入起始七关节位置
theta1=0;
theta2=90;
theta3=0;
dz4=1000;
theta5=0;
theta6=0;
theta7=-90;

%% 输入目标末端位置及旋转矩阵
% 目标末端位置 (x, y, z)
% Tpos=[100,-120,90]';     %期望位置
Tpos=[-300,300,300]';     %期望位置

% R_desired = eye(3);      % 目标姿态（单位矩阵为无旋转）

%% 绘制机械臂初始位姿及末端姿态
DHfk7Dof_Lnya(theta1,theta2,theta3,dz4,theta5,theta6,theta7,0); %绘制机械臂
hold on;
plot3(Tpos(1),Tpos(2),Tpos(3),'kX'); 
hold on;
view(132,12);
pause; cla;

%%
tic  % 记录开始时间
while (1)
    %% FK计算并绘制机器人，及目标点
    plot3(Tpos(1),Tpos(2),Tpos(3),'gx', 'MarkerSize', 8, 'LineWidth', 4); hold on;
    DHfk7Dof_Lnya(theta1,theta2,theta3,dz4,theta5,theta6,theta7,1);
    %获取机械臂末端当前位置
    ex=Link(8).p(1);
    ey=Link(8).p(2);
    ez=Link(8).p(3);
     
    %% 计算误差
    p_err =[Tpos(1)-ex, Tpos(2)-ey, Tpos(3)-ez]' ;%计算位置误差

    % w_err=[0,0,0]'; % 3DoF不考虑姿态误差，这里姿态误差置零

    % 计算姿态误差（旋转矩阵的轴角表示）
    % R_current = Link(8).R;          % 当前姿态
    % R_err = R_desired * R_current'; % 误差旋转矩阵
    % w_err = 0.5 * [R_err(3,2) - R_err(2,3);
    %     R_err(1,3) - R_err(3,1);
    %     R_err(2,1) - R_err(1,2)];
    % Loss = norm(p_err) + norm(w_err); % 综合误差 误差评价

    Loss = norm(p_err);%计算误差的欧几里得范数（L2 范数），即误差向量的长度。
    
    %% 小于期望误差则结束迭代
    if Loss<1
        break;
    end
    
    %% 否则计算雅可比矩阵并计算角度修正量
    J =Jacobian7DoF_Ln(theta1,theta2,theta3,dz4,theta5,theta6,theta7);    %计算雅可比矩阵 本次为3X7矩阵
    
    %     %%判断奇异
    %     D = det(J);
    %     if D == 0
    %         fprintf('D= %2.4f ',D); fprintf('\n');
    %         fprintf('Pass the singilarity !'); fprintf('\n');
    %         pause;
    %     end
    
    % dtheta = learning_rate * pinv(J) * [p_err; w_err];  %计算修正量，此处单位为弧度
    %%%
    % lambda = 0.01; % 设定一个小的正则化参数

    dtheta = learning_rate * (J' * pinv(J * J')) * [p_err',0,0,0]';

    
    % dtheta(5) = 200 ./ (1 + exp(-dtheta(5)));
    % dtheta(5) = min(max(dtheta(5), 0), 200);
    % dtheta(5) = 100 * (tanh(dtheta(5)) + 1);
    % fprintf('th1:%f，  th2:%f，  th3:%f，  th4:%f，  dz5:%f，  th6:%f，  th7:%f \n',dtheta')
    theta1=theta1+dtheta(1)*ToDeg;
    theta2=theta2+dtheta(2)*ToDeg;
    theta3=theta3+dtheta(3)*ToDeg;
    dz4=dz4+dtheta(4);

    % dz5=dz5+dtheta(5);
    theta5 = theta5 + (-1)^randi([0,1]) * dtheta(5);
    theta5 = 60 * (tanh(theta5) + 1);

    %%------------
    % alpha = 0.1;  % 设定插值系数
    % dz5_target = 100 * (tanh(dz5) + 1);  % 计算目标 dz5
    % dz5 = (1 - alpha) * dz5 + alpha * dz5_target;  % 线性插值平滑更新 dz5

    theta6 = theta6 + dtheta(6)*ToDeg;
    theta7 = theta7 + dtheta(7)*ToDeg;

    % count=count+1;
    % if count > 2
    %     dz5 = 100 * (tanh(dz5) + 1);
    % end
    
    fprintf('th1:%f，  th2:%f，  th3:%f，  th4:%f，  dz5:%f，  th6:%f，  th7:%f \n',Link(2).th,Link(3).th,Link(4).dz,Link(5).th,Link(6).th,Link(7).th,Link(8).th);
    % theta1=theta1+dtheta(1);
    % theta2=theta2+dtheta(2);
    % theta3=theta3+dtheta(3);
    % theta4 = theta4 + dtheta(4);
    % dz5=dz5+dtheta(5);
    % theta6 = theta6 + dtheta(6);
    % theta7 = theta7 + dtheta(7);

    x(num)=ex;
    y(num)=ey;
    z(num)=ez;
    num=num+1;
    plot3(x,y,z,'r.');
    grid on;
    hold on;    
end
toc % 记录总运行时间
%%再次绘制机器人保持图像
    fprintf('th1:%f，  th2:%f，  th3:%f，  th4:%f，  dz5:%f，  th6:%f，  th7:%f \n',Link(2).th,Link(3).th,Link(4).dz,Link(5).th,Link(6).th,Link(7).th,Link(8).th);plot3(x,y,z,'r.');grid on;
DHfk7Dof_Lnya(theta1,theta2,theta3,dz4,theta5,theta6,theta7,0);
plot3(Tpos(1),Tpos(2),Tpos(3),'kX'); hold on;






