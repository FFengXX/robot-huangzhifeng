function J=Jacobian7DoF_Ln(th1,th2,th3,th4,d5,th6,th7)
% close all
global Link

jsize=7;
J=zeros(6,jsize);

Link(2).th=th1*pi/180;
Link(3).th=th2*pi/180;
Link(4).th=th3*pi/180;
Link(5).th=th4*pi/180;
Link(6).dz=d5; % 第 5 个关节的位移
Link(7).th=th6*pi/180;
Link(8).th=th7*pi/180; % 第 7 个关节的角度
%计算那几个矩阵
for i=1:8
    Matrix_DH_Ln(i);
end

Link(1).p=Link(1).p(1:3);
for i=2:8
    Link(i).A=Link(i-1).A*Link(i).A;
    Link(i).p= Link(i).A(1:3,4);
    Link(i).n= Link(i).A(:,1);
    Link(i).o= Link(i).A(:,2);
    Link(i).a= Link(i).A(:,3);
    Link(i).R=[Link(i).n(1:3),Link(i).o(1:3),Link(i).a(1:3)];
end
%计算雅可比矩阵  
% for n=1:jsize
%     a=Link(n).R*Link(n).az;
%     J(:,n)=[cross(a,Link(8).p-Link(n).p); a];
% end
%%%%计算雅可比矩阵  
for n=1:jsize
    a = Link(n).R * Link(n).az; % % 获取关节的 Z 轴方向（旋转轴或平移轴）
    
    if n == 5 % 第 5 个关节是伸缩关节
        % 伸缩关节的雅可比矩阵列只包含平移部分，角速度部分为 0
        J(:, n) = [a; 0; 0; 0];
    else % 其他关节是旋转关节?
        % 旋转关节的雅可比矩阵列使用向量积
        J(:, n) = [cross(a, Link(8).p - Link(n).p); a];
    end
end