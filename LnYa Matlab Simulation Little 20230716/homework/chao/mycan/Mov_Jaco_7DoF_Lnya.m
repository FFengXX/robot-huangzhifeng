
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 1.0  ��������
% % ��������
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
% %% ΢���˶�
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




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%�з���
close all;
clear;
clc;

% ��������
ToDeg = 180/pi;
ToRad = pi/180;

% ��ʼ���ؽڽǶ�
th1 = 0;
th2 = 90;
th3 = 0;
th4 = 0;
d5 = 0;
th6 = 0;
th7 = 0;

% �����е�۳�ʼλ��
[initial_x, initial_y, initial_z] = DHfk7Dof_Lnya(th1, th2, th3, th4, d5, th6, th7, 1);

% ��ʼ������¼����
error_history = [];

% ���Ƴ�ʼ״̬
DHfk7Dof_Lnya(th1, th2, th3, th4, d5, th6, th7, 0);
view(134, 12);
pause;

% ΢���˶���֤
cla;
for i = 1:20
   
    DHfk7Dof_Lnya(th1, th2, th3, th4, d5, th6, th7, 0);
    hold on;
    
 
    J = Jacobian7DoF_Ln(th1, th2, th3, th4, d5, th6, th7);
    
    dD = [0; 0; 5; 0; 0; 0]; 
    
    lambda = 0.01; % ͨ��ȡ 0.01 ���С��ֵ
 dth =J' / (J * J' + lambda^2 * eye(6)) * dD ;
%     dth=pinv(J)*dD;
    
    % ���¹ؽڽǶ�
    th1 = th1 + dth(1) / pi * 180;
    th2 = th2 + dth(2) / pi * 180;
    th3 = th3 + dth(3) / pi * 180;
    th4 = th4 + dth(4) / pi * 180;
    d5 = d5 + dth(5); % �����ؽڣ�ֱ�Ӽ��ϳ��ȱ仯��
    th6 = th6 + dth(6) / pi * 180;
    th7 = th7 + dth(7) / pi * 180;
    
    % �����е��ĩ��ִ������ʵ��λ��
    [ex, ey, ez] = DHfk7Dof_Lnya(th1, th2, th3, th4, d5, th6, th7, 1);
    
    % ��������λ�ã��ڳ�ʼλ�õ� z ����ÿ�μ� 1��
    ideal_z = initial_z + i * 1; % ��ʼ z ���� + i * 1
    ideal_position = [initial_x; initial_y; ideal_z]; % ����λ�� [initial_x, initial_y, initial_z + i*1]
    
    % ����������λ�� - ʵ��λ�ã�
    p_err = norm(ideal_position - [ex; ey; ez]);
    
    % ����������
    error_history = [error_history; p_err];
    
    % ��ӡ������Ϣ
    fprintf('���� %d:\n', i);
    fprintf('����λ��: [%.4f, %.4f, %.4f]\n', initial_x, initial_y, ideal_z);
    fprintf('ʵ��λ��: [%.4f, %.4f, %.4f]\n', ex, ey, ez);
    fprintf('���: %.6f\n', p_err);
    
    % ����ĩ��ִ�����Ĺ켣
    plot3(ex, ey, ez, 'r.');
    drawnow;
    
    % ��ͣ�Թ۲��˶�
    pause(0.1);
end
hold off;

% �������仯����
figure;
plot(1:length(error_history), error_history, '-o', 'LineWidth', 1.5);
xlabel('��������');
ylabel('���');
title('���仯����');
grid on;
