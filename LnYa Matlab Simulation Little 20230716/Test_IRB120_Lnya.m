% 定义两组关节角度
angles1 = [0, 0, 0, 0, 0, 0];    % 初始姿态
angles2 = [30, -45, 60, 0, 90, 0]; % 目标姿态

% 显示初始姿态（不清除图形）
DHfk_IRB120_Lnya(angles1(1), angles1(2), angles1(3), angles1(4), angles1(5), angles1(6), false);

% 显示目标姿态（清除初始姿态）
%DHfk_IRB120_Lnya(angles2(1), angles2(2), angles2(3), angles2(4), angles2(5), angles2(6), true);