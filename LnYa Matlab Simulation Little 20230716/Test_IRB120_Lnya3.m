% 循环改变关节1角度，模拟旋转运动
for th1 = 0:10:360
    DHfk_IRB120_Lnya(th1, 0, 0, 0, 0, 0, false); % 每次清除图形
    pause(0.1); % 控制动画速度
end