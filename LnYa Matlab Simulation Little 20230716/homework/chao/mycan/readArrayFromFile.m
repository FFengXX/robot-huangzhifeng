function nijiao = readArrayFromFile(filename)
    % 读取文件中的数组（逗号分隔）
    % 输入：
    %   filename - 文件名（字符串）
    % 输出：
    %   nijiao - 一个矩阵，每行数据格式为 [value1, value2, ..., valueN]

    % 打开文件
    fileID = fopen(filename, 'r');
    if fileID == -1
        error('无法打开文件: %s', filename);
    end

    % 初始化存储数据的矩阵
    nijiao = [];

    % 逐行读取文件
    line = fgetl(fileID);
    while ischar(line)
        % 将逗号分隔的字符串转换为数值数组
        row = str2num(line); %#ok<ST2NM>
        % 将数组添加到矩阵中
        nijiao = [nijiao; row];
        % 读取下一行
        line = fgetl(fileID);
    end

    % 关闭文件
    fclose(fileID);

    % 输出矩阵，每行数据用方括号和逗号分隔
    for i = 1:size(nijiao, 1)
        fprintf('[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n', nijiao(i, :));
    end
end

