close all;
clear;

global Link
model; 

th = [0, 90, 0, 0, 0, 0,0];  
num=1;

tic;

for th1 = -180:10:180
    for th2 = -90:10:140
        for th3 = -100:10:130
            % for th4 = -300:60:300
%             for d5 = 0:40:500
                %         for th6 = -360:120:360
                 %         for th7 = -360:120:360
                DHfk7Dof_Lnya(th1,th2,th3,0,0,0,0,1);
                x(num)=Link(8).p(1);
                y(num)=Link(8).p(2);
                z(num)=Link(8).p(3);
                num=num+1;
                plot3(x,y,z,'r*');hold on;
                %                         figure(2);
                % plot(y,z,'r*');
                % xlabel('x');
                % zlabel('z');
                hold on;
                % pause(2000);
%             end
            
        end
        %     end
        % end
        %         end
    end
end

toc;

