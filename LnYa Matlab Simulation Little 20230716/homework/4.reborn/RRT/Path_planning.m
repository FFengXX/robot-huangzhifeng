function [path] = Path_planning(bengin, endp, obstacles)
    % RRT  7 ���ɶȹؽڿռ�
    
    max = 10000; % ����������?
    step = 5;          % ÿ����չ�Ĳ���
    wucha = 1;     % Ŀ�����ݲ�
    
    % ��������
    joint_limits = [
         -180, 180; 
        -90, 140;   
        -100, 130;
        -150, 170;  
        0, 500; 
        -170, 170; 
        -180, 180; 
    ];

   
    tree = bengin;         % ��ʼ����
    parents = 0;            % ���ڵ�
    explored_points = bengin; % ̽����
    
    for i = 1:max
        % �����
        sample = sampleJointPoint( endp, joint_limits);
       
        nearp = nearest_node(tree, sample);
        neartreep = tree(nearp, :);
        
        % ������ڵ� q_nearest ������� q_rand ������չһ���½ڵ� q_new?
        q_new = gen_node(neartreep, sample, step, joint_limits);
        
        % ��ײ����
        if  ~check_collision(q_new, obstacles)
            % ��?
            tree = [tree; q_new];
            parents = [parents; nearp];
            explored_points = [explored_points; q_new];
            
            % �ҵ�·��
            if norm(q_new - endp) < wucha
                fprintf('��������: %d\n', i);
                
               
                break;
            end
        end
    end
    
    % RRT �ҵ�·��
    path = endp;
    current_idx = size(tree, 1);
    while current_idx > 0
        path = [tree(current_idx, :); path];
        current_idx = parents(current_idx);
    end
    
%     disp('RRT �ҵ�·��');
end

%������ڵ� q_nearest ������� q_rand ������չһ���½ڵ� q_new
function q_new = gen_node(q_nearest, q_rand, step_size, joint_limits)
    direction = q_rand - q_nearest;
    distance = norm(direction);
    q_new = q_nearest + (direction / distance) * min(step_size, distance);
    
    % ȷ���½ڵ� q_new ��ÿ���ؽڽǶȶ��������Ʒ�Χ��
    for i = 1:length(q_new)
        q_new(i) = max(joint_limits(i,1), min(joint_limits(i,2), q_new(i)));
    end
end

% ����һ�������?
function q_rand = sampleJointPoint( goal_q, joint_limits)
    if rand < 0.9
        % �ڹؽ����Ʒ�Χ�ڵ������
        q_rand = rand(1, 7) .* (joint_limits(:,2) - joint_limits(:,1))' + joint_limits(:,1)';
    else
        % ֱ�ӵ���Ŀ���
        q_rand = goal_q;  
    end
end

%��ײ����
function in_collision = check_collision(point, obstacles)
    % Ĭ��û����ײ
    in_collision = false;
    
        % DH ���� [theta, d, a, alpha]

    DH_params = [
        0, 500, 0, pi/2;    % �ؽ� 1
        0, 0, 900, 0;         % �ؽ� 2
        0, 0, 500, pi/2;      % �ؽ� 3
        0, 500, 0, 0;      % �ؽ� 4
        0, 500, 0, -pi/2;       % �ؽ� 5
        0, 600, 0, pi/2;           % �ؽ� 6
        0, 400, 0, 0            % �ؽ� 7
    ];

    
      [~, all] = forwardKinematics(point, DH_params);
    T = double(all);
    link7_point2 = T(1:3,4,7)';
    % ���������ϰ���
    for i = 1:size(obstacles, 1)
        obs_center = obstacles(i, :);  % ��ȡ�ϰ���� 3D ����
        
        % ��������ϰ���ľ���
        if length(link7_point2) == length(obs_center)  % ȷ��ά��һ��
            % ��������ϰ������ĵ�ŷ����þ���
            if norm(link7_point2 - obs_center') < 80
                in_collision = true;  % �����ײ�ˣ�����Ϊ true
                return;
            end
        else
            error('����ĵ����ϰ���ά�Ȳ�ƥ��');
        end
    end
end


% �� RRT �����ҵ���������� q_rand ����Ľڵ�
function nearest_idx = nearest_node(tree, q_rand)
    tempDis = inf;
    for k1 = 1:size(tree, 1)
        dis = norm(q_rand - tree(k1, :));
        if tempDis > dis
            tempDis = dis;
            index = k1;
        end    
    end
    nearest_idx = index;
end

function [T, all] = forwardKinematics(joint_angles, DH_params)


    % ��ʼ���任����
    T = eye(4);
    all = zeros(4, 4, 7);

    % ����ÿ���ؽڵı任����
    for i = 1:size(DH_params, 1)
        theta = joint_angles(i) + DH_params(i, 1);  % �ؽڽǶ�
        d = DH_params(i, 2);  % ����ƫ��
        a = DH_params(i, 3);  % ���˳���
        alpha = DH_params(i, 4);  % ����Ťת��

        % ���㵱ǰ�ؽڵı任����
        Ti = [
            cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
            sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
            0,           sin(alpha),             cos(alpha),             d;
            0,           0,                       0,                      1
        ];

        % �ۻ��任����
        T = T * Ti;

        % ��¼��ǰ�ؽڵı任����
        all(:, :, i) = T;
    end
end
