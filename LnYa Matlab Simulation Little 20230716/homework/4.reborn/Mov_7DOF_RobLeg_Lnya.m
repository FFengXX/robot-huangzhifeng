close all;
clear;

ToDeg = 180/pi;
ToRad = pi/180;


th1=0;
th2=90;
th3=0;
d4=1000;
th5=0;
th6=0;
th7=-90;
DHfk7Dof_Lnya(th1,th2,th3,d4,th5,th6,th7,0);
view(134,12);
pause;
stp=15;
%% �ؽ�ת����Χ����
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Joint1
for i=-180:stp:180
   DHfk7Dof_Lnya(th1+i,th2,th3,d4,th5,th6,th7,1);
end
for i=180:-stp:-180
    if i==-180
      DHfk7Dof_Lnya(th1+i,th2,th3,d4,th5,th6,th7,0);
    else
      DHfk7Dof_Lnya(th1+i,th2,th3,d4,th5,th6,th7,1);
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Joint2


for i=-180:stp:180
   DHfk7Dof_Lnya(th1,th2+i,th3,d4,th5,th6,th7,1);
end
for i=180:-stp:-180
    if i==-180
      DHfk7Dof_Lnya(th1,th2+i,th3,d4,th5,th6,th7,0);
    else
      DHfk7Dof_Lnya(th1,th2+i,th3,d4,th5,th6,th7,1);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Joint3


for i=-180:stp:180
  DHfk7Dof_Lnya(th1,th2,th3+i,d4,th5,th6,th7,1);
end
for i=180:-stp:-180
    if i==-180
      DHfk7Dof_Lnya(th1,th2,th3+i,d4,th5,th6,th7,0);

    else
      DHfk7Dof_Lnya(th1,th2,th3+i,d4,th5,th6,th7,1);
    end 
end

d4=0;
for i=0:stp:1000
  DHfk7Dof_Lnya(th1,th2,th3,d4+i,th5,th6,th7,1);
end
for i=1000:-stp:0
    if i==0
      DHfk7Dof_Lnya(th1,th2,th3,d4+i,th5,th6,th7,0);

    else
      DHfk7Dof_Lnya(th1,th2,th3,d4+i,th5,th6,th7,1);
    end 
end
d4=1000;


for i=-200:stp:200
  DHfk7Dof_Lnya(th1,th2,th3,d4,th5+i,th6,th7,1);
end
for i=200:-stp:-200
    if i==-200
      DHfk7Dof_Lnya(th1,th2,th3,d4,th5+i,th6,th7,0);

    else
      DHfk7Dof_Lnya(th1,th2,th3,d4,th5+i,th6,th7,1);
    end 
end
for i=-200:stp:200
  DHfk7Dof_Lnya(th1,th2,th3,d4,th5,th6+i,th7,1);
end
for i=200:-stp:-200
    if i==-200
      DHfk7Dof_Lnya(th1,th2,th3,d4,th5,th6+i,th7,0);

    else
      DHfk7Dof_Lnya(th1,th2,th3,d4,th5,th6+i,th7,1);
    end 
end

for i=-200:stp:200
  DHfk7Dof_Lnya(th1,th2,th3,d4,th5,th6,th7+i,1);
end
for i=200:-stp:-200
    if i==-200
      DHfk7Dof_Lnya(th1,th2,th3,d4,th5,th6,th7+i,0);

    else
      DHfk7Dof_Lnya(th1,th2,th3,d4,th5,th6,th7+i,1);
    end 
end

% % %% ΢���˶������ſɱȾ���
% % pause;
% % cla;
% % th1=-30;
% % th2=-30;
% % th3=-30;
% % d4=30;
% % th5=30;
% % cla;
% % 
% % for  i=1:20
% %     DHfk7Dof_Lnya(th1,th2,th3,d4,th5,0);
% %     J=Jacobian7DoF_Ln(th1,th2,th3,d4,th5);
% %     dD=[0 0 0 0.1 0 0]';
% %     dth=pinv(J)*dD;
% %     th1=th1+dth(1)/pi*180;
% %     th2=th2+dth(2)/pi*180;
% %     th3=th3+dth(3)/pi*180;
% %     d4=d4+dth(4)/pi*180;
% %     th5=th5+dth(5)/pi*180;
% % end
% % 
% % %% ��ֵ������
% % %% ����Ŀ��ĩ��λ�ü���ת����
% % Tpos=[20,-30,-120]';     %����λ��
% % Trpy=[0,0,20]';     %����λ��
% % 
% 
