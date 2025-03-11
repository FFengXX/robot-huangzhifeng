% 测试多组关节角度组合
test_angles = [
    0, 0, 0, 0, 0, 0;
    90, -30, 45, 0, 60, 0;
    -45, 60, -20, 30, 90, 0;
];

for i = 1:size(test_angles, 1)
    DHfk_IRB120_Lnya(test_angles(i,1), test_angles(i,2), test_angles(i,3), ...
                    test_angles(i,4), test_angles(i,5), test_angles(i,6), false);
    pause(1); % 暂停1秒观察
end