%%%%���ؿ���
clc; clear; close all;

ToDeg = 180/pi; % ���ȵ��Ƕȵ�ת������
ToRad = pi/180; % �Ƕȵ����ȵ�ת������
UX = [1 0 0]';   % X�ᵥλ����
UY = [0 1 0]';   % Y�ᵥλ����
UZ = [0 0 1]';   % Z�ᵥλ����
% ���������ؽڵķ�Χ
z_min = 0;   % ��С��������
z_max = 500; % �����������


Link(1) = struct('name', 'Base', 'th', 0 * ToRad, 'dz', 0, 'dx', 0, 'alf', 0 * ToRad, 'az', UZ); % Base To 1
Link(2) = struct('name', 'J1', 'th', 0 * ToRad, 'dz', 500, 'dx', 0, 'alf', 90 * ToRad, 'az', UZ); % 1 TO 2
Link(3) = struct('name', 'J2', 'th', 0 * ToRad, 'dz', 0, 'dx', 900, 'alf', 0 * ToRad, 'az', UZ); % 2 TO 3
Link(4) = struct('name', 'J3', 'th', 0 * ToRad, 'dz', 0, 'dx', 500, 'alf', 90 * ToRad, 'az', UZ); % 3 TO 4  
Link(5) = struct('name', 'J4', 'th', 0 * ToRad, 'dz', 500, 'dx', 0, 'alf', 0 * ToRad, 'az', UZ); % 4 TO 5
Link(6) = struct('name', 'J5', 'th', 0 * ToRad, 'dz', 500, 'dx', 0, 'alf', -90 * ToRad, 'az', UZ); % 5 TO 6     �����ؽ�
Link(7) = struct('name', 'J6', 'th', 0 * ToRad, 'dz', 600, 'dx', 0, 'alf', 90 * ToRad, 'az', UZ); % 6 TO 7
Link(8) = struct('name', 'J7', 'th', 0 * ToRad, 'dz', 400, 'dx', 0, 'alf', 0 * ToRad, 'az', UZ); % 7 TO 8

%%3.0��100��100��0��
% �����е�۵ĹؽڽǶȷ�Χ����λ���ȣ�
th_min = [-180, -90, -100, -150, 0, -170, -180]; % �ؽڽǶȵ�����
th_max = [180,  140,  130,  170, 0,  170,  180]; % �ؽڽǶȵ�����

% ���������ؽڵĳ��ȷ�Χ
d_min = 0;   % �����ؽڵ���С����
d_max = 500; % �����ؽڵ���󳤶�

N = 50000; % ���ؿ�������������ɵ�����
workspace_points = zeros(N, 3); % Ԥ���������Դ洢�����ռ��

% �������ؿ�����������ɹ����ռ��
for b = 1:N
    % �������6���ؽڽǶȣ��ڷ�Χ�ھ��ȷֲ���
    th = th_min + (th_max - th_min) .* rand(1, 7);
    % ������������ؽڵĳ��ȣ��ڷ�Χ�ھ��ȷֲ���
    d5 = d_min + (d_max - d_min) * rand(1); % ������ؽڵĳ���

    % ����Link�ṹ�еĹؽڲ���
    for i = 1:7
        if i == 5
            Link(i+1).dz = d5; % ������ؽ��������ؽڣ����� dz
        else
            Link(i+1).th = th(i) * ToRad; % �����ؽ�����ת�ؽڣ����� th
        end
    end

    % ����ÿ�����˵����˶�ѧ
    for i = 1:8
        C = cos(Link(i).th); % �ؽڽǶȵ�����
        S = sin(Link(i).th); % �ؽڽǶȵ�����
        Ca = cos(Link(i).alf); % ����Ťת�ǵ�����
        Sa = sin(Link(i).alf); % ����Ťת�ǵ�����
        a = Link(i).dx; % ��X��ľ���
        d = Link(i).dz; % ��Z��ľ���

        % ����任�������ɲ���
        Link(i).n = [C, S, 0, 0]';
        Link(i).o = [-S*Ca, C*Ca, Sa, 0]';
        Link(i).a = [S*Sa, -C*Sa, Ca, 0]';
        Link(i).p = [a*C, a*S, d, 1]';

        % �洢��ת����ͱ任����
        Link(i).R = [Link(i).n(1:3), Link(i).o(1:3), Link(i).a(1:3)];
        Link(i).A = [Link(i).n, Link(i).o, Link(i).a, Link(i).p];
    end

    % ����ÿ������ĩ�˵�λ��
    for i = 2:8
        Link(i).A = Link(i-1).A * Link(i).A; % �����任����
        Link(i).p = Link(i).A(:, 4); % ��ȡĩ��ִ������λ��
    end

    % �洢ĩ��ִ������λ��
    workspace_points(b, :) = Link(8).p(1:3)';
end

% ���ƻ�е�۵Ĺ����ռ�
figure;
hold on; % ���ֵ�ǰͼ��
scatter3(workspace_points(:,1), workspace_points(:,2), workspace_points(:,3), 2, 'b', 'filled');
xlabel('X'); ylabel('Y'); zlabel('Z');
title('IRB 1300-12/1.4 - ���ؿ�����������ռ�');
grid on; axis equal;

% ������ӵĳߴ�
box_length = 2000; % ��
box_width = 1000;  % ��
box_height = 2000; % ��
box_x_start = -1000; % ���ӵ� x �����
box_y_start = -500; % ���ӵ� y �����

% ������ӵĶ�������
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

% ������ӵ��棨ÿ������4��������ɣ�
faces = [
    1, 2, 3, 4; % ����
    5, 6, 7, 8; % ����
    1, 2, 6, 5; % ǰ��
    2, 3, 7, 6; % ����
    3, 4, 8, 7; % ����
    4, 1, 5, 8; % ����
];

% ���ƺ���
patch('Vertices', vertices, 'Faces', faces, 'FaceColor', 'none', 'EdgeColor', 'r', 'LineWidth', 1.5);

% ����10��С��
num_spheres = 10; % С������
sphere_radius = 50; % С��İ뾶
[x, y, z] = sphere(20); % ����С��ļ�������

% �������С���λ��
for i = 1:num_spheres
    % �������С�����ĵ�λ��
    center_x = box_x_start + rand() * box_length; % ���� x ����
    center_y = box_y_start + rand() * box_width;  % ���� y ����
    center_z = rand() * box_height;

    % ����С��
    surf(x * sphere_radius + center_x, ...
         y * sphere_radius + center_y, ...
         z * sphere_radius + center_z, ...
         'EdgeColor', 'none', 'FaceColor', 'g'); % ��ɫС��
end

% ����ͼ������
xlabel('X');
ylabel('Y');
zlabel('Z');
title('�����ռ�');
axis equal;
grid on;
view(3); % ����Ϊ 3D �ӽ�
hold off;