%%%%%%%%%%%%����rrt

% ��ʼ�ؽ�
start_q = [0, 90, 0, 0, 0, 0, 0];  

radius = 40;
fileID = fopen('�ϰ���.txt', 'r');   % ���ļ�
obstacles = textscan(fileID, '%f %f %f', 'Delimiter', ',');  % ���ж�ȡ���֣���ָ���ָ���Ϊ����
fclose(fileID);  % �ر��ļ�

% ����ȡ������ת��Ϊ����
obstacles = cell2mat(obstacles);
%%%%%%%%%%%%%%%%%%%%6.0
% ��ȡС��λ��
% ��ȡС��λ��  С����ʧ�ɹ�
qiuwzhi = readArrayFromFile('�ϰ���.txt');

% ������ӵĳߴ����ʼλ��
box_length = 1000;        % �� = max(x) - min(x) = 1532 - 532 = 1000
box_width  = 2000;        % �� = max(y) - min(y) = 677 - (-1323) = 2000
box_height = 2000;        % �� = max(z) - min(z) = 2139 - 139 = 2000
box_x_start = 532;        % ���ӵ� x ����� (min(x))
box_y_start = -1323;      % ���ӵ� y ����� (min(y))
box_z_start = 139;        % ���ӵ� z ����� (min(z))
% ������İ뾶
sphere_radius = 50;

% ����ͼ�δ���
figure;
hold on;
grid on;
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('��е�۹켣');
view(3);

% ������ӵĶ���
vertices = [
    box_x_start, box_y_start, box_z_start;
    box_x_start + box_length, box_y_start, box_z_start;
    box_x_start + box_length, box_y_start + box_width, box_z_start;
    box_x_start, box_y_start + box_width, box_z_start;
    box_x_start, box_y_start, box_z_start + box_height;
    box_x_start + box_length, box_y_start, box_z_start + box_height;
    box_x_start + box_length, box_y_start + box_width, box_z_start + box_height;
    box_x_start, box_y_start + box_width, box_z_start + box_height;
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
patch('Vertices', vertices, 'Faces', faces, 'FaceColor', 'none', 'FaceAlpha', 1, 'EdgeColor', 'blue');

% ����С��
[x, y, z] = sphere(20); % ����С��ļ�������
ball_handles = []; % �洢С���ͼ�ξ��
for i = 1:size(qiuwzhi, 1)
    % ��ȡС���λ��
    center_x = qiuwzhi(i, 1);
    center_y = qiuwzhi(i, 2);
    center_z = qiuwzhi(i, 3);

    % ����С�򲢴洢���
    ball_handles(i) = surf(x * sphere_radius + center_x, ...
                           y * sphere_radius + center_y, ...
                           z * sphere_radius + center_z, ...
                           'EdgeColor', 'y', 'FaceColor', 'y');
end

% ����ͼ������
axis([box_x_start, box_x_start + box_length, ...
      box_y_start, box_y_start + box_width, ...
      box_z_start, box_z_start + box_height]);

% ��ȡĿ���
filename = '�����Ƕ�.txt';
nijiao = readArrayFromFile(filename);
disp(nijiao);

% ����ӽ�С�����ֵ
threshold = 10; % ����е��ĩ����С��ľ���С�ڴ�ֵʱ����Ϊ����С��λ��

% ���е��� RRT4 ����
for i = 1:size(nijiao, 1)
    % ��ȡ�� i ����ΪĿ���
    goal_q = nijiao(i, :);
    
    % ���� RRT4 ����
    [path] = Path_planning(start_q, goal_q, obstacles);
    founditerations = [];
    
    % �������·���е�ÿ���ڵ㣨����ʼ�㵽Ŀ��㣩
    for j = 1:size(path, 1)
        target_angles = path(j, :);
        
        % ���¹ؽڽǶ�
        th1 = target_angles(1);
        th2 = target_angles(2);
        th3 = target_angles(3);
        th4 = target_angles(4);
        th5 = target_angles(5);
        th6 = target_angles(6);
        th7 = target_angles(7);  
        
        % ʹ���˶�ѧģ�ͼ����е��ĩ��ִ����λ��
        points = DHfk7Dof_Lnya2(th1, th2, th3, th4, th5, th6, th7,1);  
        end_effector_x = points(1);
        end_effector_y = points(2);
        end_effector_z = points(3);

        
        % ���»���С��ͺ���
        cla; % �����ǰ���ڵ�����ͼ��
        patch('Vertices', vertices, 'Faces', faces, 'FaceColor', 'none', 'FaceAlpha', 0.1, 'EdgeColor', 'blue');
        
        % ����е��ĩ���Ƿ�ӽ�С��
        for k = size(qiuwzhi, 1):-1:1
            center_x = qiuwzhi(k, 1);
            center_y = qiuwzhi(k, 2);
            center_z = qiuwzhi(k, 3);
            
            % �����е��ĩ����С��ľ���
            distance = sqrt((end_effector_x - center_x)^2 + ...
                            (end_effector_y - center_y)^2 + ...
                            (end_effector_z - center_z)^2);
            
            % �������С����ֵ���Ƴ�С��
            if distance < threshold
                if ishandle(ball_handles(k)) % ������Ƿ���Ч
                    delete(ball_handles(k)); % ɾ��С���ͼ��
                end
                qiuwzhi(k, :) = []; % �Ӿ������Ƴ�С���λ��
                ball_handles(k) = []; % �Ӿ���������Ƴ�С��ľ��
                drawnow; % ��������ͼ��
            else
                % �������»���С��
                surf(x * sphere_radius + center_x, ...
                     y * sphere_radius + center_y, ...
                     z * sphere_radius + center_z, ...
                     'EdgeColor', 'y', 'FaceColor', 'y');
            end
        end
    end

    % �ص�ԭ�㣬�������·��
    for j = size(path, 1):-1:1
        target_angles = path(j, :);
        
        % ���¹ؽڽǶ�
        th1 = target_angles(1);
        th2 = target_angles(2);
        th3 = target_angles(3);
        th4 = target_angles(4);
        th5 = target_angles(5);
        th6 = target_angles(6);
        th7 = target_angles(7);  
        
        % ʹ���˶�ѧģ�ͼ����е��ĩ��ִ����λ��
        points = DHfk7Dof_Lnya2(th1, th2, th3, th4, th5, th6, th7,1);  
        end_effector_x = points(1);
        end_effector_y = points(2);
        end_effector_z = points(3);
        % ���»���С��ͺ���
        cla; % �����ǰ���ڵ�����ͼ��
        patch('Vertices', vertices, 'Faces', faces, 'FaceColor', 'none', 'FaceAlpha', 0.1, 'EdgeColor', 'blue');
        
        % ����е��ĩ���Ƿ�ӽ�С��
        for k = size(qiuwzhi, 1):-1:1
            center_x = qiuwzhi(k, 1);
            center_y = qiuwzhi(k, 2);
            center_z = qiuwzhi(k, 3);
            
            % �����е��ĩ����С��ľ���
            distance = sqrt((end_effector_x - center_x)^2 + ...
                            (end_effector_y - center_y)^2 + ...
                            (end_effector_z - center_z)^2);
            
            % �������С����ֵ���Ƴ�С��
            if distance < threshold
                if ishandle(ball_handles(k)) % ������Ƿ���Ч
                    delete(ball_handles(k)); % ɾ��С���ͼ��
                end
                qiuwzhi(k, :) = []; % �Ӿ������Ƴ�С���λ��
                ball_handles(k) = []; % �Ӿ���������Ƴ�С��ľ��
                drawnow; % ��������ͼ��
            else
                % �������»���С��
                surf(x * sphere_radius + center_x, ...
                     y * sphere_radius + center_y, ...
                     z * sphere_radius + center_z, ...
                     'EdgeColor', 'y', 'FaceColor', 'y');
            end
        end
    end
end

hold off;
