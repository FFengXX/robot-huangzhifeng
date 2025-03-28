% A(1:90)=struct('cdata',[],'colormap',[]);
clear;

global Link
Build_7DOFRobot_Lnya();


tol = 10;  % 设定一个容差
threshold_z = 100;  % 设定z坐标的阈值
% pause;


% 预先计算总迭代次数并预分配空间（如果能计算出较准确的点数，可以预分配以加快速度）
theta1_vals = -180:60:180;  
theta2_vals = -180:60:180;  
theta3_vals = -180:60:180;  
dz4_vals =     0:100:1200;
theta5_vals = -200:60:200;   
theta6_vals = -200:60:200;  

totalPoints = numel(theta1_vals)*numel(theta2_vals)*numel(theta3_vals)*numel(dz4_vals)*numel(theta5_vals)*numel(theta6_vals);

% 假设 Link(6).p 前三维为x,y,z，这里预分配一个 3 x totalPoints 的矩阵
points = zeros(3, totalPoints);
count = 1;
tic;
for theta1 = theta1_vals
    for theta2 = theta2_vals
        for theta3 = theta3_vals
            for theta4 = dz4_vals
                for dz5 = theta5_vals
                    for theta6 = theta6_vals
                        Link(2).th = theta1;
                        Link(3).th = theta2;
                        Link(4).th = theta3;
                        Link(5).dz = theta4;
                        Link(6).th = dz5;
                        Link(7).th = theta6;

                        % 调用每个链节的 DH 参数计算函数
                        for i = 1:8
                            Matrix_DH_Ln(i);
                        end
                        for i = 2:8
                            Link(i).A = Link(i-1).A * Link(i).A;
                            Link(i).p = Link(i).A(:,4);
                        end

                        % % 保存计算出的末端点
                        points(:, count) = Link(8).p(1:3);
                        count = count + 1;

                        % 仅保存z大于阈值的点
                        % if Link(8).p(3) > threshold_z
                        %     points(:, count) = Link(8).p(1:3);
                        %     count = count + 1;
                        % end
                    end
                end
            end
        end
    end
end

% 移除未填充的部分，避免过大的预分配
points = points(:, 1:count-1);
toc;
close all;

% 循环结束后一次性绘制所有点
% figure;
% 创建 Figure 1（如果已存在，则直接使用）
% figure(1);
% clf; % 清空图像，避免旧数据干扰
% plot3(points(1,:), points(2,:), points(3,:), 'r*');
% grid on;
% rotate3d on;
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title('机械臂工作空间');
% axis equal;
% axis auto;

%%-------------------------------------------
% 画出完整的机械臂工作空间 (Figure 1)
figure(1);  % 创建 Figure 1
clf;  % 清除 Figure 1 之前的内容
plot3(points(1,:), points(2,:), points(3,:), 'r*');  % 绘制 3D 散点图
grid on;
rotate3d on;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('机械臂工作空间');
axis equal;
axis auto;

%%-------------
% % 判断 y 坐标是否在容差范围内，并绘制 x-z 切面 (Figure 2)
% figure(2);  % 创建 Figure 2
% clf;  % 清除 Figure 2 之前的内容
% hold on;
% grid on;
% 
% % 选取满足条件的点
% idx = abs(points(2,:)+20) < tol;
% if any(idx)  % 如果有符合条件的点
%     plot(points(1, idx), points(3, idx), 'r*');  % 仅绘制 x-z 平面上的点
% end
% xlabel('X');
% ylabel('Z');
% title('x-z 坐标切面');
% axis equal;
% hold off;
% 
% 
% % 判断 y 坐标是否在容差范围内 (Figure 3)
% figure(3);  % 创建 Figure 3
% clf;  % 清除 Figure 3 之前的内容
% hold on;
% grid on;
% rotate3d on;
% 
% % 选取满足条件的点
% idx = abs(points(2,:)+20) < tol;
% if any(idx)  % 如果有符合条件的点
%     plot3(points(1,idx), points(2,idx), points(3,idx), 'r*');  % 绘制 3D 散点图
% 
% end
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title('y在tol中的工作空间点');
% axis equal;
% axis(-500,500,-500,500,-600,600)
% hold off;


%%-----------------剖面图
tol = 10;  % 容差设定

%% ------------- X = 0 时的 Y-Z 剖面图 (Figure 2) -------------
figure(2);
clf;
hold on;
grid on;

idx_x0 = abs(points(1, :)) < tol;  % 筛选 x ≈ 0 的点
if any(idx_x0)
    plot(points(2, idx_x0), points(3, idx_x0), 'r*');  % 绘制 Y-Z 平面
end
xlabel('Y');
ylabel('Z');
title('X=0 的 Y-Z 剖面图');
axis equal;
hold off;

%% ------------- Y = 0 时的 X-Z 剖面图 (Figure 3) -------------
figure(3);
clf;
hold on;
grid on;

idx_y0 = abs(points(2, :)) < tol;  % 筛选 y ≈ 0 的点
if any(idx_y0)
    plot(points(1, idx_y0), points(3, idx_y0), 'g*');  % 绘制 X-Z 平面
end
xlabel('X');
ylabel('Z');
title('Y=0 的 X-Z 剖面图');
axis equal;
hold off;

%% ------------- Z = 0 时的 X-Y 剖面图 (Figure 4) -------------
figure(4);
clf;
hold on;
grid on;

idx_z0 = abs(points(3, :)) < tol;  % 筛选 z ≈ 0 的点
if any(idx_z0)
    plot(points(1, idx_z0), points(2, idx_z0), 'b*');  % 绘制 X-Y 平面
end
xlabel('X');
ylabel('Y');
title('Z=0 的 X-Y 剖面图');
axis equal;
hold off;