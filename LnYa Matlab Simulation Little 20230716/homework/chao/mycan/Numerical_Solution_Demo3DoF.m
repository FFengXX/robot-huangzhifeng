%%%%%%%%%%%%随机生成的
clc;
clear all;
close all;

global Link 
num=1;
learning_rate = 0.5;  %影响了机械臂关节角度调整的步长和收敛速度  收敛速度较快，但可能会出现震荡    
ToDeg = 180/pi;
ToRad = pi/180;
 z_min = 0; z_max = 500; % 伸缩关节范围
th_min = [-180, -90, -100, -150, 0, -170, -180]; % 关节角度的下限
th_max = [180,  140,  130,  170, 0,  170,  180]; % 关节角度的上限


%%%%%3.0（100，100，0）
% 定义盒子的尺寸
box_length = 2000; % 长
box_width = 1000;  % 宽
box_height = 2000; % 高
box_x_start = -1000; % 盒子的 x 轴起点
box_y_start = 500; % 盒子的 y 轴起点

% 生成10个随机目标点
num_targets = 10;
Tpos_list = zeros(3, num_targets);
for i = 1:num_targets
    Tpos_list(1, i) = box_x_start + rand() * box_length;  % x坐标在 [100, 2100] 范围内
    Tpos_list(2, i) = box_y_start + rand() * box_width;   % y坐标在 [100, 1100] 范围内
    Tpos_list(3, i) = rand() * box_height;  % z坐标在 [0, 2000] 范围内
end

% 打印生成的目标点
disp('生成的目标点坐标：');
for i = 1:num_targets
    fprintf('目标点 %d: [%.2f, %.2f, %.2f]\n', i, Tpos_list(1, i), Tpos_list(2, i), Tpos_list(3, i));
end

% 绘制盒子
figure;
hold on;
grid on;
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');

% 定义盒子的顶点
vertices = [
    box_x_start, box_y_start, 0;
    box_x_start + box_length, box_y_start, 0;
    box_x_start + box_length, box_y_start + box_width, 0;
    box_x_start, box_y_start + box_width, 0;
    box_x_start, box_y_start, box_height;
    box_x_start + box_length, box_y_start, box_height;
    box_x_start + box_length, box_y_start + box_width, box_height;
    box_x_start, box_y_start + box_width, box_height;
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
patch('Vertices', vertices, 'Faces', faces, 'FaceColor', 'red', 'FaceAlpha', 0.1, 'EdgeColor', 'black');

% 绘制目标点（用小球代替）
for i = 1:num_targets
    scatter3(Tpos_list(1, i), Tpos_list(2, i), Tpos_list(3, i), 100, 'filled', 'b');
end

% 文件路径
target_filename = fullfile(pwd, 'win.txt');   % 存储目标位置
joint_filename = fullfile(pwd, 'win01.txt');  % 存储关节角度

% 清空文件内容（如果文件存在的话）
if exist(target_filename, 'file')
    delete(target_filename);
end
if exist(joint_filename, 'file')
    delete(joint_filename);
end

% 创建新的空文件
target_file = fopen(target_filename, 'w');
joint_file = fopen(joint_filename, 'w');

if target_file == -1 || joint_file == -1
    error('无法创建文件');
end

% 对每个目标点进行逆运动学求解
for i = 1:num_targets
    Tpos = Tpos_list(:, i);  % 获取当前目标点
    
    % 初始化关节角度
    th1 = 0;
    th2 = 90;
    th3 = 0;
    th4 = 0;
    d5 = 0;
    th6 = 0;
    th7 = 0;
    
    % 绘制初始状态
    DHfk7Dof_Lnya(th1, th2, th3, th4, d5, th6, th7, 0);
    scatter3(Tpos(1), Tpos(2), Tpos(3), 100, 'filled', 'b'); hold on;
    view(-21, 12);
    pause; cla;
    
    % 进行逆运动学求解
    tic; % 开始计时
    while (1)
        % 检查是否超过20秒
        if toc > 20
            fprintf('目标点 %d: 超过20秒未收敛，跳转到下一个点\n', i);
            break;
        end
        
        scatter3(Tpos(1), Tpos(2), Tpos(3), 100, 'filled', 'b'); hold on;
        DHfk7Dof_Lnya(th1, th2, th3, th4, d5, th6, th7, 1)
        
        ex = Link(8).p(1);
        ey = Link(8).p(2);
        ez = Link(8).p(3);
        
        p_err = [Tpos(1) - ex, Tpos(2) - ey, Tpos(3) - ez]';
        w_err = [0, 0, 0]';
        Loss = norm(p_err) + norm(w_err);
        
        if Loss < 1e-4
            fprintf('目标点 %d: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n', i, th1, th2, th3, th4, d5, th6, th7);
                        elapsed_time = toc; % 获取当前时间
            fprintf('目标点 %d: 收敛用时 %.2f 秒\n', i, elapsed_time);
            fprintf('目标点 %d: 误差 %.6f\n', i, Loss);
            fprintf('目标点 %d: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n', i, th1, th2, th3, th4, d5, th6, th7);
            
            % 如果成功，则将目标点和关节角度写入文件
            fprintf(target_file, '%.4f, %.4f, %.4f\n', Tpos(1), Tpos(2), Tpos(3));  % 存储目标位置
            fprintf(joint_file, '%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n', th1, th2, th3, th4, d5, th6, th7);  % 存储关节角度
            break;
        end
        
        J = Jacobian7DoF_Ln(th1, th2, th3, th4, d5, th6, th7);
        dth = learning_rate * pinv(J) * [p_err; w_err];
        
        th1 = min(max(th1 + dth(1) * ToDeg, th_min(1)), th_max(1));
        th2 = min(max(th2 + dth(2) * ToDeg, th_min(2)), th_max(2));
        th3 = min(max(th3 + dth(3) * ToDeg, th_min(3)), th_max(3));
        th4 = min(max(th4 + dth(4) * ToDeg, th_min(4)), th_max(4));
        d5 = min(max(d5 + dth(5), z_min), z_max);
        th6 = min(max(th6 + dth(6) * ToDeg, th_min(6)), th_max(6));
        th7 = min(max(th7 + dth(7) * ToDeg, th_min(7)), th_max(7));
        
        x(num) = ex;
        y(num) = ey;
        z(num) = ez;
        num = num + 1;
        plot3(x, y, z, 'r.'); grid on;
        hold on;
    end
    
    % 绘制最终状态
    plot3(x, y, z, 'r.'); grid on;
    DHfk7Dof_Lnya(th1, th2, th3, th4, d5, th6, th7, 0)
    scatter3(Tpos(1), Tpos(2), Tpos(3), 100, 'filled', 'b'); hold on;
end

% 关闭文件
fclose(target_file);
fclose(joint_file);




















%%%%固定那10个点
% clc;
% clear all;
% close all;
% 
% global Link 
% num=1;
% learning_rate = 0.5;  %影响了机械臂关节角度调整的步长和收敛速度  收敛速度较快，但可能会出现震荡    
% ToDeg = 180/pi;
% ToRad = pi/180;
% z_min = 0; z_max = 500; % 伸缩关节范围
% th_min = [-180, -90, -100, -150, 0, -170, -180]; % 关节角度的下限
% th_max = [180,  140,  130,  170, 0,  170,  180]; % 关节角度的上限
% 
% % 文件路径
% obstacle_filename = fullfile(pwd, '障碍物.txt');   % 存储障碍物位置（目标点）
% 
% % 打开文件并检查是否成功
% obstacle_file = fopen(obstacle_filename, 'r');
% if obstacle_file == -1
%     error('无法打开文件 %s', obstacle_filename);
% end
% 
% % 读取目标点坐标
% Tpos_list = [];
% while ~feof(obstacle_file)
%     line = fgetl(obstacle_file);  % 逐行读取文件
%     if ischar(line)
%         coords = str2num(line);  % 将字符串转换为数字
%         if numel(coords) == 3  % 确保是3个数，表示目标点坐标
%             Tpos_list = [Tpos_list, coords'];  % 存储到目标点列表
%         end
%     end
% end
% fclose(obstacle_file);  % 关闭文件
% 
% % 打印读取的目标点
% disp('读取的目标点坐标：');
% for i = 1:size(Tpos_list, 2)
%     fprintf('目标点 %d: [%.2f, %.2f, %.2f]\n', i, Tpos_list(1, i), Tpos_list(2, i), Tpos_list(3, i));
% end
% 
% % 绘制盒子
% figure;
% hold on;
% grid on;
% axis equal;
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% 
% % 定义盒子的尺寸
% box_length = 2000; % 长
% box_width = 1000;  % 宽
% box_height = 2000; % 高
% box_x_start = -1000; % 盒子的 x 轴起点
% box_y_start = 500; % 盒子的 y 轴起点
% 
% % 定义盒子的顶点
% vertices = [
%     box_x_start, box_y_start, 0;
%     box_x_start + box_length, box_y_start, 0;
%     box_x_start + box_length, box_y_start + box_width, 0;
%     box_x_start, box_y_start + box_width, 0;
%     box_x_start, box_y_start, box_height;
%     box_x_start + box_length, box_y_start, box_height;
%     box_x_start + box_length, box_y_start + box_width, box_height;
%     box_x_start, box_y_start + box_width, box_height;
% ];
% 
% % 定义盒子的面
% faces = [
%     1, 2, 3, 4; % 底面
%     5, 6, 7, 8; % 顶面
%     1, 2, 6, 5; % 前面
%     2, 3, 7, 6; % 右面
%     3, 4, 8, 7; % 后面
%     4, 1, 5, 8  % 左面
% ];
% 
% % 绘制盒子
% patch('Vertices', vertices, 'Faces', faces, 'FaceColor', 'red', 'FaceAlpha', 0.1, 'EdgeColor', 'black');
% 
% % 绘制目标点（用小球代替）
% for i = 1:size(Tpos_list, 2)
%     scatter3(Tpos_list(1, i), Tpos_list(2, i), Tpos_list(3, i), 100, 'filled', 'b');
% end
% 
% % 文件路径
% target_filename = fullfile(pwd, 'win.txt');   % 存储目标位置
% joint_filename = fullfile(pwd, 'win01.txt');  % 存储关节角度
% 
% % 清空文件内容（如果文件存在的话）
% if exist(target_filename, 'file')
%     delete(target_filename);
% end
% if exist(joint_filename, 'file')
%     delete(joint_filename);
% end
% 
% % 创建新的空文件
% target_file = fopen(target_filename, 'w');
% joint_file = fopen(joint_filename, 'w');
% 
% if target_file == -1 || joint_file == -1
%     error('无法创建文件');
% end
% 
% % 对每个目标点进行逆运动学求解
% for i = 1:size(Tpos_list, 2)
%     Tpos = Tpos_list(:, i);  % 获取当前目标点
%     
%     % 初始化关节角度
%     th1 = 0;
%     th2 = 90;
%     th3 = 0;
%     th4 = 0;
%     d5 = 0;
%     th6 = 0;
%     th7 = 0;
%     
%     % 绘制初始状态
%     DHfk7Dof_Lnya(th1, th2, th3, th4, d5, th6, th7, 0);
%     scatter3(Tpos(1), Tpos(2), Tpos(3), 100, 'filled', 'b'); hold on;
%     view(-21, 12);
%     pause; cla;
%     
%     % 进行逆运动学求解
%     tic; % 开始计时
%     while (1)
%         % 检查是否超过20秒
%         if toc > 20
%             fprintf('目标点 %d: 超过20秒未收敛，跳转到下一个点\n', i);
%             break;
%         end
%         
%         scatter3(Tpos(1), Tpos(2), Tpos(3), 100, 'filled', 'b'); hold on;
%         DHfk7Dof_Lnya(th1, th2, th3, th4, d5, th6, th7, 1)
%         
%         ex = Link(8).p(1);
%         ey = Link(8).p(2);
%         ez = Link(8).p(3);
%         
%         p_err = [Tpos(1) - ex, Tpos(2) - ey, Tpos(3) - ez]';
%         w_err = [0, 0, 0]';
%         Loss = norm(p_err) + norm(w_err);
%         
%         if Loss < 1e-4
%             elapsed_time = toc; % 获取当前时间
%             fprintf('目标点 %d: 收敛用时 %.2f 秒\n', i, elapsed_time);
%             fprintf('目标点 %d: 误差 %.6f\n', i, Loss);
%             fprintf('目标点 %d: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n', i, th1, th2, th3, th4, d5, th6, th7);
%             
%             % 如果成功，则将目标点和关节角度写入文件
%             fprintf(target_file, '%.4f, %.4f, %.4f\n', Tpos(1), Tpos(2), Tpos(3));  % 存储目标位置
%             fprintf(joint_file, '%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n', th1, th2, th3, th4, d5, th6, th7);  % 存储关节角度
%             break;
%         end
%         
%         J = Jacobian7DoF_Ln(th1, th2, th3, th4, d5, th6, th7);
%         dth = learning_rate * pinv(J) * [p_err; w_err];
%         
%         th1 = min(max(th1 + dth(1) * ToDeg, th_min(1)), th_max(1));
%         th2 = min(max(th2 + dth(2) * ToDeg, th_min(2)), th_max(2));
%         th3 = min(max(th3 + dth(3) * ToDeg, th_min(3)), th_max(3));
%         th4 = min(max(th4 + dth(4) * ToDeg, th_min(4)), th_max(4));
%         d5 = min(max(d5 + dth(5), z_min), z_max);
%         th6 = min(max(th6 + dth(6) * ToDeg, th_min(6)), th_max(6));
%         th7 = min(max(th7 + dth(7) * ToDeg, th_min(7)), th_max(7));
%         
%         x(num) = ex;
%         y(num) = ey;
%         z(num) = ez;
%         num = num + 1;
%         plot3(x, y, z, 'r.'); grid on;
%         hold on;
%     end
%     % 绘制最终状态
%     plot3(x, y, z, 'r.'); grid on;
%     DHfk7Dof_Lnya(th1, th2, th3, th4, d5, th6, th7, 0);
%     scatter3(Tpos(1), Tpos(2), Tpos(3), 100, 'filled', 'b'); hold on;
% end
% 
% 
% % 关闭文件
% fclose(target_file);
% fclose(joint_file);
% 
% disp('目标点与关节角度数据已保存至文件。');



