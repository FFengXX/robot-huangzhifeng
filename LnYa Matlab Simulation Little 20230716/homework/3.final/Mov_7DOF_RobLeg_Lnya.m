close all;
clear;

ToDeg = 180/pi;
ToRad = pi/180;


th1=0;
th2=90;
th3=0;
th4=0;
th5=0;
th6=0;
th7=0;
DHfk7Dof_Lnya(th1,th2,th3,th4,th5,th6,th7,0);
view(134,12);
pause;
stp=30;
% %% 关节转动范围测试
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Joint1
% for i=-180:stp:180
%    DHfk6Dof_Lnya(th1+i,th2,th3,th4,th5,th6,1);
% end
% for i=180:-stp:-180
%     if i==-180
%       DHfk6Dof_Lnya(th1+i,th2,th3,th4,th5,th6,0);
%     else
%       DHfk6Dof_Lnya(th1+i,th2,th3,th4,th5,th6,1);
%     end
% end
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Joint2
% 
% 
% for i=-90:stp:150
%    DHfk6Dof_Lnya(th1,th2+i,th3,th4,th5,th6,1);
% end
% for i=150:-stp:-90
%     if i==-90
%       DHfk6Dof_Lnya(th1,th2+i,th3,th4,th5,th6,0);
%     else
%       DHfk6Dof_Lnya(th1,th2+i,th3,th4,th5,th6,1);
%     end
% end
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Joint3
% 
% 
% for i=-75:stp:180
%   DHfk6Dof_Lnya(th1,th2,th3+i,th4,th5,th6,1);
% end
% for i=180:-stp:-75
%     if i==-75
%       DHfk6Dof_Lnya(th1,th2,th3+i,th4,th5,th6,0);
% 
%     else
%       DHfk6Dof_Lnya(th1,th2,th3+i,th4,th5,th6,1);
%     end 
% end
% 
% for i=-400:stp:400
%   DHfk6Dof_Lnya(th1,th2,th3,th4+i,th5,th6,1);
% end
% for i=400:-stp:-400
%     if i==-400
%       DHfk6Dof_Lnya(th1,th2,th3,th4+i,th5,th6,0);
% 
%     else
%       DHfk6Dof_Lnya(th1,th2,th3,th4+i,th5,th6,1);
%     end 
% end
% 
% for i=-125:stp:120
%   DHfk6Dof_Lnya(th1,th2,th3,th4,th5+i,th6,1);
% end
% for i=120:-stp:-125
%     if i==-125
%       DHfk6Dof_Lnya(th1,th2,th3,th4,th5+i,th6,0);
% 
%     else
%       DHfk6Dof_Lnya(th1,th2,th3,th4,th5+i,th6,1);
%     end 
% end
% 
% for i=-400:stp:400
%   DHfk5Dof_Lnya(th1,th2,th3,th4,th5,th6+i,1);
% end
% for i=400:-stp:-400
%     if i==-400
%       DHfk6Dof_Lnya(th1,th2,th3,th4,th5,th6+i,0);
% 
%     else
%       DHfk6Dof_Lnya(th1,th2,th3,th4,th5,th6+i,1);
%     end 
% end
% %% 微分运动测试雅可比矩阵
% pause;
% cla;
% th1=-30;
% th2=-30;
% th3=-30;
% th4=30;
% th5=30;
% cla;
% 
% for  i=1:20
%     DHfk5Dof_Lnya(th1,th2,th3,th4,th5,0);
%     J=Jacobian5DoF_Ln(th1,th2,th3,th4,th5);
%     dD=[0 0 0 0.1 0 0]';
%     dth=pinv(J)*dD;
%     th1=th1+dth(1)/pi*180;
%     th2=th2+dth(2)/pi*180;
%     th3=th3+dth(3)/pi*180;
%     th4=th4+dth(4)/pi*180;
%     th5=th5+dth(5)/pi*180;
% end
% 
% %% 数值逆解测试
% %% 输入目标末端位置及旋转矩阵
% Tpos=[20,-30,-120]';     %期望位置
% Trpy=[0,0,20]';     %期望位置


