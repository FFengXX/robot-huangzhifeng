ToDeg = 180/pi;
ToRad = pi/180; % 角度弧度互相转化
UX = [1 0 0]';
UY = [0 1 0]';
UZ = [0 0 1]'; % 单位向量

% 参数对齐说明：
% name   th       dz     dx     alf      az
% θ      th       绕 Z 轴的旋转角度（关节角）
% d      dz       沿 Z 轴的平移距离（连杆偏移）
% a      dx       沿 X 轴的平移距离（连杆长度）
% α      alf      绕 X 轴的旋转角度（连杆扭转角）
%--------------------------------------------
Link = struct('name', 'Body',  'th', 0,      'dz', 0,    'dx', 0,    'alf', 0*ToRad,  'az', UZ);  % 定义这个结构体

Link(1) = struct('name', 'Base',  'th', 0*ToRad,  'dz', 0,    'dx', 0,    'alf', 0*ToRad,  'az', UZ);  % Base To 1
Link(2) = struct('name', 'J1',    'th', 0*ToRad, 'dz', 500,  'dx', 0,  'alf', 90*ToRad, 'az', UZ);  % 1 TO 2
Link(3) = struct('name', 'J2',    'th', 90*ToRad, 'dz', 0,    'dx', 900,  'alf', 90*ToRad,        'az', UZ);  % 2 TO 3
Link(4) = struct('name', 'J3',    'th', 0*ToRad,  'dz', 500,  'dx', 0,    'alf', -90*ToRad,'az', UZ);  % 4 TO 5
Link(5) = struct('name', 'J4',    'th', 0*ToRad,  'dz', 500,    'dx', 0,    'alf', 90*ToRad, 'az', UZ);  % 5 TO 6
Link(6) = struct('name', 'J5',    'th', 0*ToRad,  'dz', 500,  'dx', 0,    'alf', 90*ToRad,  'az', UZ);  % 6 TO 7
Link(7) = struct('name', 'J6',    'th', -90*ToRad,  'dz', 0,  'dx', 1000,    'alf', -90*ToRad,  'az', UZ);  % 7 TO 8
Link(8) = struct('name', 'Footf', 'th', 0*ToRad,'dz', 400,   'dx', 0,    'alf', 0*ToRad,  'az', UZ);  % 末端

% Link(9) = struct('name', 'Footb', 'th', 0*ToRad,  'dz', 0,    'dx', 60,   'alf', 0*ToRad,  'az', UZ);  % Footb（注释）