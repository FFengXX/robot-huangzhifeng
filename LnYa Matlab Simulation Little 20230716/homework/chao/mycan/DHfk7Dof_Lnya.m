% function DHfk7Dof_Lnya(th1,th2,th3,th4,d5,th6,th7,fcla)
% % close all
% 
% global Link
% 
% model;
% radius    = 25;  %25
% len       = 60;  %60
% joint_col = 0;
% 
% 
% plot3(0,0,0,'ro'); 
% 
% 
%   Link(2).th= th1*pi/180;
%  Link(3).th=th2*pi/180;
%  Link(4).th=th3*pi/180;
%  Link(5).th=th4*pi/180;
% %  Link(6).th=th5*pi/180;
%  
%  Link(6).dz = Link(5).dz + d5;
%  Link(7).th=th6*pi/180;
%  Link(8).th = th7*pi/180;
% 
% p0=[0,0,0]';
% 
% 
% for i=1:8
% Matrix_DH_Ln(i);
% end
% 
% 
% for i=2:8
% 
%       Link(i).A=Link(i-1).A*Link(i).A;
%       Link(i).p= Link(i).A(:,4);
%       Link(i).n= Link(i).A(:,1);
%       Link(i).o= Link(i).A(:,2);
%       Link(i).a= Link(i).A(:,3);
%       Link(i).R=[Link(i).n(1:3),Link(i).o(1:3),Link(i).a(1:3)];
%       Connect3D(Link(i-1).p,Link(i).p,'b',2); hold on;
%        plot3(Link(i).p(1),Link(i).p(2),Link(i).p(3),'rx');hold on;
%       if i<=8
%           DrawCylinder(Link(i-1).p, Link(i-1).R * Link(i).az, radius,len, joint_col); hold on;
%       end 
% end
% % view(125,52);
% % set (gcf,'Position',[650,100,700,600])
% axis([-2500,2500,-2500,2500,-2500,3000]);
% xlabel('x');
% ylabel('y'); 
% zlabel('z');
% grid on;
% drawnow;
% if(fcla)
%     cla;
% end


function [end_effector_x, end_effector_y, end_effector_z] = DHfk7Dof_Lnya(th1, th2, th3, th4, d5, th6, th7, fcla)
% �������ܣ������е��ĩ��ִ������λ��
% ���룺
%   th1, th2, th3, th4, th6, th7: �ؽڽǶȣ���λ���ȣ�
%   d5: �ؽ�5��λ��
%   fcla: �Ƿ����ͼ�δ��ڣ�1: �����0: �������
% �����
%   end_effector_x, end_effector_y, end_effector_z: ĩ��ִ������λ��

global Link

Build_7DOFRobot_Lnya;
radius    = 25;  %25
len       = 60;  %60
joint_col = 0;

plot3(0, 0, 0, 'ro'); 

% ���ùؽڽǶȺ�λ��
Link(2).th = th1 * pi / 180;
Link(3).th = th2 * pi / 180;
Link(4).th = th3 * pi / 180;
Link(5).th = th4 * pi / 180;
Link(6).dz = Link(5).dz + d5;
Link(7).th = th6 * pi / 180;
Link(8).th = th7 * pi / 180;

% ����ÿ�����˵ı任����
for i = 1:8
    Matrix_DH_Ln(i);
end

% �����е��ĩ��ִ������λ��
for i = 2:8
    Link(i).A = Link(i - 1).A * Link(i).A;
    Link(i).p = Link(i).A(:, 4);
    Link(i).n = Link(i).A(:, 1);
    Link(i).o = Link(i).A(:, 2);
    Link(i).a = Link(i).A(:, 3);
    Link(i).R = [Link(i).n(1:3), Link(i).o(1:3), Link(i).a(1:3)];
    Connect3D(Link(i - 1).p, Link(i).p, 'b', 2); hold on;
    plot3(Link(i).p(1), Link(i).p(2), Link(i).p(3), 'rx'); hold on;
    if i <= 8
        DrawCylinder(Link(i - 1).p, Link(i - 1).R * Link(i).az, radius, len, joint_col); hold on;
    end
end

% ����ͼ������
axis([-2500, 2500, -2500, 2500, -2500, 3000]);
xlabel('x');
ylabel('y');
zlabel('z');
grid on;
drawnow;

% ����ĩ��ִ������λ��
end_effector_x = Link(8).p(1);
end_effector_y = Link(8).p(2);
end_effector_z = Link(8).p(3);

% �Ƿ����ͼ�δ���
if fcla
    cla;
end
end



