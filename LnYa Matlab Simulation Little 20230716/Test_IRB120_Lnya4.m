try
    % 输入非数值参数（如字符串）
    DHfk_IRB120_Lnya('invalid', 0, 0, 0, 0, 0, true);
catch ME
    disp('错误信息:');
    disp(ME.message);
end