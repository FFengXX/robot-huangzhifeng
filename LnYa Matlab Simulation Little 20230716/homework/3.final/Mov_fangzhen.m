close all;
clear;


global Link
Build_6DOFRobot_Lnya; 

th = [0, 90, 0, 0, 0, 0]; % 初始关节角度
num=1;
% 开始计时
tic;

for th1 = 1:1:1
    for th2 = 0:10:270
        for d3 = 0:100:1000
            % for th4 = -300:60:300
            % for th5 = -130:65:130
            % for th6 = -360:120:360
            % if((th2==25 && th3>-90 && th3<=160) ||(th3==-90 && th2>25 && th2<175)||(th2==175 && th3>-90 && th3<160)||(th3==90 && th3>=25 && th3<=175)||(th3==160 && th2>=25 && th2<=175))
            DHfk6Dof_Lnya(th1,th2,d3,0,0,0,0,1); 
            x(num)=Link(7).p(1);
            y(num)=Link(7).p(2);
            z(num)=Link(7).p(3);
            num=num+1;
            plot3(x,y,z,'r*');hold on;
            % figure(2);
            % plot(y,z,'r*');
            % xlabel('x');
            % zlabel('z');
            hold on;
            % end
            % end
            % end
            % end
        end
    end
end

% 结束计时并显示花费的时间
toc;