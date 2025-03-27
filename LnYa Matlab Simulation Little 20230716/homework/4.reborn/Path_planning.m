function [path] = Path_planning(bengin, endp, obstacles)
    % RRT  7 自由度关节空间
    
    max = 10000; % 最大迭代次数?
    step = 5;          % 每次扩展的步长
    wucha = 1;     % 目标点的容差
    
    % 根据区间
    joint_limits = [
         -180, 180; 
        -90, 140;   
        -100, 130;
        -150, 170;  
        0, 500; 
        -170, 170; 
        -180, 180; 
    ];

   
    tree = bengin;         % 开始的树
    parents = 0;            % 父节点
    explored_points = bengin; % 探索点
    
    for i = 1:max
        % 随机点
        sample = sampleJointPoint( endp, joint_limits);
       
        nearp = nearest_node(tree, sample);
        neartreep = tree(nearp, :);
        
        % 从最近节点 q_nearest 向随机点 q_rand 方向扩展一个新节点 q_new?
        q_new = gen_node(neartreep, sample, step, joint_limits);
        
        % 碰撞测试
        if  ~check_collision(q_new, obstacles)
            % 树?
            tree = [tree; q_new];
            parents = [parents; nearp];
            explored_points = [explored_points; q_new];
            
            % 找到路径
            if norm(q_new - endp) < wucha
                fprintf('迭代次数: %d\n', i);
                
               
                break;
            end
        end
    end
    
    % RRT 找到路径
    path = endp;
    current_idx = size(tree, 1);
    while current_idx > 0
        path = [tree(current_idx, :); path];
        current_idx = parents(current_idx);
    end
    
%     disp('RRT 找到路径');
end

%从最近节点 q_nearest 向随机点 q_rand 方向扩展一个新节点 q_new
function q_new = gen_node(q_nearest, q_rand, step_size, joint_limits)
    direction = q_rand - q_nearest;
    distance = norm(direction);
    q_new = q_nearest + (direction / distance) * min(step_size, distance);
    
    % 确保新节点 q_new 的每个关节角度都在其限制范围内
    for i = 1:length(q_new)
        q_new(i) = max(joint_limits(i,1), min(joint_limits(i,2), q_new(i)));
    end
end

% 生成一个随机点?
function q_rand = sampleJointPoint( goal_q, joint_limits)
    if rand < 0.9
        % 在关节限制范围内的随机点
        q_rand = rand(1, 7) .* (joint_limits(:,2) - joint_limits(:,1))' + joint_limits(:,1)';
    else
        % 直接等于目标点
        q_rand = goal_q;  
    end
end

%碰撞处理
function in_collision = check_collision(point, obstacles)
    % 默认没有碰撞
    in_collision = false;
    
        % DH 参数 [theta, d, a, alpha]

    DH_params = [
        0, 500, 0, pi/2;    % 关节 1
        0, 0, 900, 0;         % 关节 2
        0, 0, 500, pi/2;      % 关节 3
        0, 500, 0, 0;      % 关节 4
        0, 500, 0, -pi/2;       % 关节 5
        0, 600, 0, pi/2;           % 关节 6
        0, 400, 0, 0            % 关节 7
    ];

    
      [~, all] = forwardKinematics(point, DH_params);
    T = double(all);
    link7_point2 = T(1:3,4,7)';
    % 遍历所有障碍物
    for i = 1:size(obstacles, 1)
        obs_center = obstacles(i, :);  % 获取障碍物的 3D 坐标
        
        % 计算点与障碍物的距离
        if length(link7_point2) == length(obs_center)  % 确保维度一致
            % 计算点与障碍物中心的欧几里得距离
            if norm(link7_point2 - obs_center') < 80
                in_collision = true;  % 如果碰撞了，设置为 true
                return;
            end
        else
            error('输入的点与障碍物维度不匹配');
        end
    end
end


% 在 RRT 树中找到距离随机点 q_rand 最近的节点
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


    % 初始化变换矩阵
    T = eye(4);
    all = zeros(4, 4, 7);

    % 计算每个关节的变换矩阵
    for i = 1:size(DH_params, 1)
        theta = joint_angles(i) + DH_params(i, 1);  % 关节角度
        d = DH_params(i, 2);  % 连杆偏移
        a = DH_params(i, 3);  % 连杆长度
        alpha = DH_params(i, 4);  % 连杆扭转角

        % 计算当前关节的变换矩阵
        Ti = [
            cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
            sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
            0,           sin(alpha),             cos(alpha),             d;
            0,           0,                       0,                      1
        ];

        % 累积变换矩阵
        T = T * Ti;

        % 记录当前关节的变换矩阵
        all(:, :, i) = T;
    end
end
