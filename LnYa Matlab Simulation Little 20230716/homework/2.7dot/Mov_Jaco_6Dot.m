close all;
clear;
% 通过正运动学分析当前状态，并通过逆运动学迭代调整关节角度，实现末端执行器的定向运动
% 初始的参数只有th1-th6

% 这个脚本循环做了三件事
% 正运动学计算：
% 调用 DHfk_J_Puma560_Lnya，根据当前关节角度绘制机械臂，并得到末端位置。

% 逆运动学求解：
% 调用 Jacobian6DoF_Ln 计算雅可比矩阵 J，然后通过 雅可比逆矩阵 将期望的末端位移 dD = [20, 0, 0, 0, 0, 0]'（沿 X 轴移动 20 单位）转换为关节角度增量 dth：

% 更新关节角度：
% 将 dth 转换为角度并叠加到当前关节角度，使末端逐步向目标方向移动。



figure; 

 th1=0;
 th2=90;
 th3=0;
 th4=0;
 th5=0;
 th6=0;


DHfk6Dof_Lnya(th1,th2,th3,th4,th5,th6,0); %
pause;
num=1;
for i=1:20
    figure(1);   
    DHfk6Dof_Lnya(th1,th2,th3,th4,th5,th6,0);%
    J=Jacobian6DoF_Ln(th1,th2,th3,th4,th5,th6); 
    x=det(J) ;fprintf('x= %2.4f ',x);  
    xout(num)=x;
    t(num)=i;
    num=num+1;
    
    % dD=[20 0 0 0 0 0]';
    dD=[80 0 0 0 0 0]';
    dth=inv(J)*dD;
    th1=th1+dth(1)/pi*180;
    th2=th2+dth(2)/pi*180;
    th3=th3+dth(3)/pi*180;
    th4=th4+dth(4)/pi*180;
    th5=th5+dth(5)/pi*180;
    th6=th6+dth(6)/pi*180;
    
    % if i~=100
    %   DHfk_J_Puma560_Lnya(th1,th2,th3,th4,th5,th6,0);
    % else
    %   DHfk_J_Puma560_Lnya(th1,th2,th3,th4,th5,th6,0);
    % end
end
figure(3);
axis([0,num,-5000000,5000000]);
plot(t,xout,'r-O');hold on;



