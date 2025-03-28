clc;
clear all;
close all;

global Link 
num=1;
learning_rate = 2;
lambda = 0.01;
title('��ֵ�����--ѧϰ��:2--���򻯲���:0.01');
rotate3d on;
hold on;

ToDeg = 180/pi;
ToRad = pi/180;
count = 0;

%������ʼ�߹ؽ�λ��
theta1=0;
theta2=90;
theta3=0;
dz4=1000;
theta5=0;
theta6=0;
theta7=-90;

%% ����Ŀ��ĩ��λ�ü���ת����
% Ŀ��ĩ��λ�� (x, y, z)
% Tpos=[100,-120,90]';     %����λ��
Tpos=[-300,300,300]';     %����λ��

% R_desired = eye(3);      % Ŀ����̬����λ����Ϊ����ת��

%% ���ƻ�е�۳�ʼλ�˼�ĩ����̬
DHfk7Dof_Lnya(theta1,theta2,theta3,dz4,theta5,theta6,theta7,0); %���ƻ�е��
hold on;
plot3(Tpos(1),Tpos(2),Tpos(3),'kX'); 
hold on;
view(132,12);
pause; cla;

%%
tic  % ��¼��ʼʱ��
while (1)
    %% FK���㲢���ƻ����ˣ���Ŀ���
    plot3(Tpos(1),Tpos(2),Tpos(3),'gx', 'MarkerSize', 8, 'LineWidth', 4); hold on;
    DHfk7Dof_Lnya(theta1,theta2,theta3,dz4,theta5,theta6,theta7,1);
    %��ȡ��е��ĩ�˵�ǰλ��
    ex=Link(8).p(1);
    ey=Link(8).p(2);
    ez=Link(8).p(3);
     
    %% �������
    p_err =[Tpos(1)-ex, Tpos(2)-ey, Tpos(3)-ez]' ;%����λ�����

    % w_err=[0,0,0]'; % 3DoF��������̬��������̬�������

    % ������̬����ת�������Ǳ�ʾ��
    % R_current = Link(8).R;          % ��ǰ��̬
    % R_err = R_desired * R_current'; % �����ת����
    % w_err = 0.5 * [R_err(3,2) - R_err(2,3);
    %     R_err(1,3) - R_err(3,1);
    %     R_err(2,1) - R_err(1,2)];
    % Loss = norm(p_err) + norm(w_err); % �ۺ���� �������

    Loss = norm(p_err);%��������ŷ����÷�����L2 ������������������ĳ��ȡ�
    
    %% С������������������
    if Loss<1
        break;
    end
    
    %% ��������ſɱȾ��󲢼���Ƕ�������
    J =Jacobian7DoF_Ln(theta1,theta2,theta3,dz4,theta5,theta6,theta7);    %�����ſɱȾ��� ����Ϊ3X7����
    
    %     %%�ж�����
    %     D = det(J);
    %     if D == 0
    %         fprintf('D= %2.4f ',D); fprintf('\n');
    %         fprintf('Pass the singilarity !'); fprintf('\n');
    %         pause;
    %     end
    
    % dtheta = learning_rate * pinv(J) * [p_err; w_err];  %�������������˴���λΪ����
    %%%
    % lambda = 0.01; % �趨һ��С�����򻯲���

    dtheta = learning_rate * (J' * pinv(J * J')) * [p_err',0,0,0]';

    
    % dtheta(5) = 200 ./ (1 + exp(-dtheta(5)));
    % dtheta(5) = min(max(dtheta(5), 0), 200);
    % dtheta(5) = 100 * (tanh(dtheta(5)) + 1);
    % fprintf('th1:%f��  th2:%f��  th3:%f��  th4:%f��  dz5:%f��  th6:%f��  th7:%f \n',dtheta')
    theta1=theta1+dtheta(1)*ToDeg;
    theta2=theta2+dtheta(2)*ToDeg;
    theta3=theta3+dtheta(3)*ToDeg;
    dz4=dz4+dtheta(4);

    % dz5=dz5+dtheta(5);
    theta5 = theta5 + (-1)^randi([0,1]) * dtheta(5);
    theta5 = 60 * (tanh(theta5) + 1);

    %%------------
    % alpha = 0.1;  % �趨��ֵϵ��
    % dz5_target = 100 * (tanh(dz5) + 1);  % ����Ŀ�� dz5
    % dz5 = (1 - alpha) * dz5 + alpha * dz5_target;  % ���Բ�ֵƽ������ dz5

    theta6 = theta6 + dtheta(6)*ToDeg;
    theta7 = theta7 + dtheta(7)*ToDeg;

    % count=count+1;
    % if count > 2
    %     dz5 = 100 * (tanh(dz5) + 1);
    % end
    
    fprintf('th1:%f��  th2:%f��  th3:%f��  th4:%f��  dz5:%f��  th6:%f��  th7:%f \n',Link(2).th,Link(3).th,Link(4).dz,Link(5).th,Link(6).th,Link(7).th,Link(8).th);
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
toc % ��¼������ʱ��
%%�ٴλ��ƻ����˱���ͼ��
    fprintf('th1:%f��  th2:%f��  th3:%f��  th4:%f��  dz5:%f��  th6:%f��  th7:%f \n',Link(2).th,Link(3).th,Link(4).dz,Link(5).th,Link(6).th,Link(7).th,Link(8).th);plot3(x,y,z,'r.');grid on;
DHfk7Dof_Lnya(theta1,theta2,theta3,dz4,theta5,theta6,theta7,0);
plot3(Tpos(1),Tpos(2),Tpos(3),'kX'); hold on;






