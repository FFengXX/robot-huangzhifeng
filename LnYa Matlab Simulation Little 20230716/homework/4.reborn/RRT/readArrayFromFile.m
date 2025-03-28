function nijiao = readArrayFromFile(filename)
    % ��ȡ�ļ��е����飨���ŷָ���
    % ���룺
    %   filename - �ļ������ַ�����
    % �����
    %   nijiao - һ������ÿ�����ݸ�ʽΪ [value1, value2, ..., valueN]

    % ���ļ�
    fileID = fopen(filename, 'r');
    if fileID == -1
        error('�޷����ļ�: %s', filename);
    end

    % ��ʼ���洢���ݵľ���
    nijiao = [];

    % ���ж�ȡ�ļ�
    line = fgetl(fileID);
    while ischar(line)
        % �����ŷָ����ַ���ת��Ϊ��ֵ����
        row = str2num(line); %#ok<ST2NM>
        % ��������ӵ�������
        nijiao = [nijiao; row];
        % ��ȡ��һ��
        line = fgetl(fileID);
    end

    % �ر��ļ�
    fclose(fileID);

    % �������ÿ�������÷����źͶ��ŷָ�
    for i = 1:size(nijiao, 1)
        fprintf('[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n', nijiao(i, :));
    end
end

