function Matrix_DH_Ln(i) 
% Caculate the D-H Matrix
%基于 DH 参数计算第 i 个连杆的坐标系变换矩阵
global Link

ToDeg = 180/pi;
ToRad = pi/180;


C=cos(Link(i).th);    %cosθi
S=sin(Link(i).th);    %sinθi
Ca=cos(Link(i).alf);  %cosαi
Sa=sin(Link(i).alf);  %sinαi
a=Link(i).dx;         %a               %distance between zi and zi-1
d=Link(i).dz;         %d               %distance between xi and xi-1

Link(i).n=[C,S,0,0]';
Link(i).o=[-1*S*Ca,C*Ca,Sa,0]';
Link(i).a=[S*Sa, -1*C*Sa,Ca,0]';
Link(i).p=[a*C,a*S,d,1]';           %关节i到关节i+1的坐标变换

%旋转矩阵     描述连杆的姿态（旋转），用于计算速度、加速度或坐标变换
Link(i).R=[Link(i).n(1:3),Link(i).o(1:3),Link(i).a(1:3)];

%齐次变换矩阵 同时描述连杆的 ​姿态（旋转）​ 和 ​位置（平移）​，用于正运动学计算
Link(i).A=[Link(i).n,Link(i).o,Link(i).a,Link(i).p];     