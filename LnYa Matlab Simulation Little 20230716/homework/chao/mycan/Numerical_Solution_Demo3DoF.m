%%%%%%%%%%%%������ɵ�
clc;
clear all;
close all;

global Link 
num=1;
learning_rate = 0.5;  %Ӱ���˻�е�۹ؽڽǶȵ����Ĳ����������ٶ�  �����ٶȽϿ죬�����ܻ������    
ToDeg = 180/pi;
ToRad = pi/180;
 z_min = 0; z_max = 500; % �����ؽڷ�Χ
th_min = [-180, -90, -100, -150, 0, -170, -180]; % �ؽڽǶȵ�����
th_max = [180,  140,  130,  170, 0,  170,  180]; % �ؽڽǶȵ�����


%%%%%3.0��100��100��0��
% ������ӵĳߴ�
box_length = 2000; % ��
box_width = 1000;  % ��
box_height = 2000; % ��
box_x_start = -1000; % ���ӵ� x �����
box_y_start = 500; % ���ӵ� y �����

% ����10�����Ŀ���
num_targets = 10;
Tpos_list = zeros(3, num_targets);
for i = 1:num_targets
    Tpos_list(1, i) = box_x_start + rand() * box_length;  % x������ [100, 2100] ��Χ��
    Tpos_list(2, i) = box_y_start + rand() * box_width;   % y������ [100, 1100] ��Χ��
    Tpos_list(3, i) = rand() * box_height;  % z������ [0, 2000] ��Χ��
end

% ��ӡ���ɵ�Ŀ���
disp('���ɵ�Ŀ������꣺');
for i = 1:num_targets
    fprintf('Ŀ��� %d: [%.2f, %.2f, %.2f]\n', i, Tpos_list(1, i), Tpos_list(2, i), Tpos_list(3, i));
end

% ���ƺ���
figure;
hold on;
grid on;
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');

% ������ӵĶ���
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

% ������ӵ���
faces = [
    1, 2, 3, 4; % ����
    5, 6, 7, 8; % ����
    1, 2, 6, 5; % ǰ��
    2, 3, 7, 6; % ����
    3, 4, 8, 7; % ����
    4, 1, 5, 8  % ����
];

% ���ƺ���
patch('Vertices', vertices, 'Faces', faces, 'FaceColor', 'red', 'FaceAlpha', 0.1, 'EdgeColor', 'black');

% ����Ŀ��㣨��С����棩
for i = 1:num_targets
    scatter3(Tpos_list(1, i), Tpos_list(2, i), Tpos_list(3, i), 100, 'filled', 'b');
end

% �ļ�·��
target_filename = fullfile(pwd, 'win.txt');   % �洢Ŀ��λ��
joint_filename = fullfile(pwd, 'win01.txt');  % �洢�ؽڽǶ�

% ����ļ����ݣ�����ļ����ڵĻ���
if exist(target_filename, 'file')
    delete(target_filename);
end
if exist(joint_filename, 'file')
    delete(joint_filename);
end

% �����µĿ��ļ�
target_file = fopen(target_filename, 'w');
joint_file = fopen(joint_filename, 'w');

if target_file == -1 || joint_file == -1
    error('�޷������ļ�');
end

% ��ÿ��Ŀ���������˶�ѧ���
for i = 1:num_targets
    Tpos = Tpos_list(:, i);  % ��ȡ��ǰĿ���
    
    % ��ʼ���ؽڽǶ�
    th1 = 0;
    th2 = 90;
    th3 = 0;
    th4 = 0;
    d5 = 0;
    th6 = 0;
    th7 = 0;
    
    % ���Ƴ�ʼ״̬
    DHfk7Dof_Lnya(th1, th2, th3, th4, d5, th6, th7, 0);
    scatter3(Tpos(1), Tpos(2), Tpos(3), 100, 'filled', 'b'); hold on;
    view(-21, 12);
    pause; cla;
    
    % �������˶�ѧ���
    tic; % ��ʼ��ʱ
    while (1)
        % ����Ƿ񳬹�20��
        if toc > 20
            fprintf('Ŀ��� %d: ����20��δ��������ת����һ����\n', i);
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
            fprintf('Ŀ��� %d: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n', i, th1, th2, th3, th4, d5, th6, th7);
                        elapsed_time = toc; % ��ȡ��ǰʱ��
            fprintf('Ŀ��� %d: ������ʱ %.2f ��\n', i, elapsed_time);
            fprintf('Ŀ��� %d: ��� %.6f\n', i, Loss);
            fprintf('Ŀ��� %d: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n', i, th1, th2, th3, th4, d5, th6, th7);
            
            % ����ɹ�����Ŀ���͹ؽڽǶ�д���ļ�
            fprintf(target_file, '%.4f, %.4f, %.4f\n', Tpos(1), Tpos(2), Tpos(3));  % �洢Ŀ��λ��
            fprintf(joint_file, '%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n', th1, th2, th3, th4, d5, th6, th7);  % �洢�ؽڽǶ�
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
    
    % ��������״̬
    plot3(x, y, z, 'r.'); grid on;
    DHfk7Dof_Lnya(th1, th2, th3, th4, d5, th6, th7, 0)
    scatter3(Tpos(1), Tpos(2), Tpos(3), 100, 'filled', 'b'); hold on;
end

% �ر��ļ�
fclose(target_file);
fclose(joint_file);




















%%%%�̶���10����
% clc;
% clear all;
% close all;
% 
% global Link 
% num=1;
% learning_rate = 0.5;  %Ӱ���˻�е�۹ؽڽǶȵ����Ĳ����������ٶ�  �����ٶȽϿ죬�����ܻ������    
% ToDeg = 180/pi;
% ToRad = pi/180;
% z_min = 0; z_max = 500; % �����ؽڷ�Χ
% th_min = [-180, -90, -100, -150, 0, -170, -180]; % �ؽڽǶȵ�����
% th_max = [180,  140,  130,  170, 0,  170,  180]; % �ؽڽǶȵ�����
% 
% % �ļ�·��
% obstacle_filename = fullfile(pwd, '�ϰ���.txt');   % �洢�ϰ���λ�ã�Ŀ��㣩
% 
% % ���ļ�������Ƿ�ɹ�
% obstacle_file = fopen(obstacle_filename, 'r');
% if obstacle_file == -1
%     error('�޷����ļ� %s', obstacle_filename);
% end
% 
% % ��ȡĿ�������
% Tpos_list = [];
% while ~feof(obstacle_file)
%     line = fgetl(obstacle_file);  % ���ж�ȡ�ļ�
%     if ischar(line)
%         coords = str2num(line);  % ���ַ���ת��Ϊ����
%         if numel(coords) == 3  % ȷ����3��������ʾĿ�������
%             Tpos_list = [Tpos_list, coords'];  % �洢��Ŀ����б�
%         end
%     end
% end
% fclose(obstacle_file);  % �ر��ļ�
% 
% % ��ӡ��ȡ��Ŀ���
% disp('��ȡ��Ŀ������꣺');
% for i = 1:size(Tpos_list, 2)
%     fprintf('Ŀ��� %d: [%.2f, %.2f, %.2f]\n', i, Tpos_list(1, i), Tpos_list(2, i), Tpos_list(3, i));
% end
% 
% % ���ƺ���
% figure;
% hold on;
% grid on;
% axis equal;
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% 
% % ������ӵĳߴ�
% box_length = 2000; % ��
% box_width = 1000;  % ��
% box_height = 2000; % ��
% box_x_start = -1000; % ���ӵ� x �����
% box_y_start = 500; % ���ӵ� y �����
% 
% % ������ӵĶ���
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
% % ������ӵ���
% faces = [
%     1, 2, 3, 4; % ����
%     5, 6, 7, 8; % ����
%     1, 2, 6, 5; % ǰ��
%     2, 3, 7, 6; % ����
%     3, 4, 8, 7; % ����
%     4, 1, 5, 8  % ����
% ];
% 
% % ���ƺ���
% patch('Vertices', vertices, 'Faces', faces, 'FaceColor', 'red', 'FaceAlpha', 0.1, 'EdgeColor', 'black');
% 
% % ����Ŀ��㣨��С����棩
% for i = 1:size(Tpos_list, 2)
%     scatter3(Tpos_list(1, i), Tpos_list(2, i), Tpos_list(3, i), 100, 'filled', 'b');
% end
% 
% % �ļ�·��
% target_filename = fullfile(pwd, 'win.txt');   % �洢Ŀ��λ��
% joint_filename = fullfile(pwd, 'win01.txt');  % �洢�ؽڽǶ�
% 
% % ����ļ����ݣ�����ļ����ڵĻ���
% if exist(target_filename, 'file')
%     delete(target_filename);
% end
% if exist(joint_filename, 'file')
%     delete(joint_filename);
% end
% 
% % �����µĿ��ļ�
% target_file = fopen(target_filename, 'w');
% joint_file = fopen(joint_filename, 'w');
% 
% if target_file == -1 || joint_file == -1
%     error('�޷������ļ�');
% end
% 
% % ��ÿ��Ŀ���������˶�ѧ���
% for i = 1:size(Tpos_list, 2)
%     Tpos = Tpos_list(:, i);  % ��ȡ��ǰĿ���
%     
%     % ��ʼ���ؽڽǶ�
%     th1 = 0;
%     th2 = 90;
%     th3 = 0;
%     th4 = 0;
%     d5 = 0;
%     th6 = 0;
%     th7 = 0;
%     
%     % ���Ƴ�ʼ״̬
%     DHfk7Dof_Lnya(th1, th2, th3, th4, d5, th6, th7, 0);
%     scatter3(Tpos(1), Tpos(2), Tpos(3), 100, 'filled', 'b'); hold on;
%     view(-21, 12);
%     pause; cla;
%     
%     % �������˶�ѧ���
%     tic; % ��ʼ��ʱ
%     while (1)
%         % ����Ƿ񳬹�20��
%         if toc > 20
%             fprintf('Ŀ��� %d: ����20��δ��������ת����һ����\n', i);
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
%             elapsed_time = toc; % ��ȡ��ǰʱ��
%             fprintf('Ŀ��� %d: ������ʱ %.2f ��\n', i, elapsed_time);
%             fprintf('Ŀ��� %d: ��� %.6f\n', i, Loss);
%             fprintf('Ŀ��� %d: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n', i, th1, th2, th3, th4, d5, th6, th7);
%             
%             % ����ɹ�����Ŀ���͹ؽڽǶ�д���ļ�
%             fprintf(target_file, '%.4f, %.4f, %.4f\n', Tpos(1), Tpos(2), Tpos(3));  % �洢Ŀ��λ��
%             fprintf(joint_file, '%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n', th1, th2, th3, th4, d5, th6, th7);  % �洢�ؽڽǶ�
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
%     % ��������״̬
%     plot3(x, y, z, 'r.'); grid on;
%     DHfk7Dof_Lnya(th1, th2, th3, th4, d5, th6, th7, 0);
%     scatter3(Tpos(1), Tpos(2), Tpos(3), 100, 'filled', 'b'); hold on;
% end
% 
% 
% % �ر��ļ�
% fclose(target_file);
% fclose(joint_file);
% 
% disp('Ŀ�����ؽڽǶ������ѱ������ļ���');



