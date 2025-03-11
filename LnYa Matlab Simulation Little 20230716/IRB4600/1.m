close all;
clear;

ToDeg = 180/pi;
ToRad = pi/180;

% 初始关节角度（单位：度）
th1 = 0;
th2 = 90;
th3 = 0;
th4 = 0;
th5 = 0;
th6 = 0;

% 调用六轴机械臂的前向运动学函数
DHfk6Dof_Lnya(th1, th2, th3, th4, th5, th6, 0);
view(134, 12);
pause;

stp = 5;

% % 关节1的转动范围测试
%  for i = -170:stp:170
%      DHfk6Dof_Lnya(th1 + i, th2, th3, th4, th5, th6, 1);
%  end
%  for i = 170:-stp:-170
%      if i == -170
%          DHfk6Dof_Lnya(th1 + i, th2, th3, th4, th5, th6, 0);
%      else
%          DHfk6Dof_Lnya(th1 + i, th2, th3, th4, th5, th6, 1);
%      end
%  end
% 
% % 关节2的转动范围测试
%  for i = -65:stp:85
%      DHfk6Dof_Lnya(th1, th2 + i, th3, th4, th5, th6, 1);
%  end
%  for i = 85:-stp:-65
%      if i == -65
%          DHfk6Dof_Lnya(th1, th2 + i, th3, th4, th5, th6, 0);
%      else
%          DHfk6Dof_Lnya(th1, th2 + i, th3, th4, th5, th6, 1);
%      end
%  end
% 
% % 关节3的转动范围测试
% for i = -70:stp:180
%     DHfk6Dof_Lnya(th1, th2, th3 + i, th4, th5, th6, 1);
% 
% end
% for i = 180:-stp:-70
%     if i == -70
%         DHfk6Dof_Lnya(th1, th2, th3 + i, th4, th5, th6, 0);
%     else
%         DHfk6Dof_Lnya(th1, th2, th3 + i, th4, th5, th6, 1);
%     end
% end
% 
% % 关节4的转动范围测试
% for i = -300:stp:300
%     DHfk6Dof_Lnya(th1, th2, th3, th4 + i, th5, th6, 1);
% 
% end
% for i = 300:-stp:-300
%     if i == -300
%         DHfk6Dof_Lnya(th1, th2, th3, th4 + i, th5, th6, 0);
%     else
%         DHfk6Dof_Lnya(th1, th2, th3, th4 + i, th5, th6, 1);
%     end
% end
% 
% % 关节5的转动范围测试
%  for i = -130:stp:130
%      DHfk6Dof_Lnya(th1, th2, th3, th4, th5 + i, th6, 1);
%  end
%  for i = 130:-stp:-130
%      if i == -130
%          DHfk6Dof_Lnya(th1, th2, th3, th4, th5 + i, th6, 0);
%      else
%          DHfk6Dof_Lnya(th1, th2, th3, th4, th5 + i, th6, 1);
%      end
%  end
% 
% %% 关节6的转动范围测试
%  for i = -360:stp:360
%      DHfk6Dof_Lnya(th1, th2, th3, th4, th5, th6 + i, 1);
%  end
%  for i = 360:-stp:-360
%      if i == -360
%          DHfk6Dof_Lnya(th1, th2, th3, th4, th5, th6 + i, 0);
%      else
%          DHfk6Dof_Lnya(th1, th2, th3, th4, th5, th6 + i, 1);
%      end
%  end

%% 微分运动测试雅可比矩阵
% pause;
% cla;
% th1 = -30;
% th2 = -30;
% th3 = -30;
% th4 = 30;
% th5 = 30;
% th6 = 30;
% cla;
% 
% for i = 1:20
%     DHfk6Dof_Lnya(th1, th2, th3, th4, th5, th6, 0);
%     J = Jacobian6DoF_Ln(th1, th2, th3, th4, th5, th6);
%     dD = [0 0 0 0.1 0 0]';
%     dth = pinv(J) * dD;
%     th1 = th1 + dth(1)/pi*180;
%     th2 = th2 + dth(2)/pi*180;
%     th3 = th3 + dth(3)/pi*180;
%     th4 = th4 + dth(4)/pi*180;
%     th5 = th5 + dth(5)/pi*180;
%     th6 = th6 + dth(6)/pi*180;
% end

%% 数值逆解测试
% 输入目标末端位置及旋转矩阵
% Tpos = [20, -30, -120]';  % 期望位置
% Trpy = [0, 0, 20]';  % 期望姿态
