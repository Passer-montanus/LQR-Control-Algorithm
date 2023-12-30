% 路径跟踪横向LQR仿真主函数

clc
clear
close all
addpath  Calc_fcns Draw_fcns Limited&Update %载入函数文件夹
%% 1. 设置车辆初始状态
veh_pose = [0, -15, 0];       %车辆初始位姿
steer_state = 0;            %当前时刻前轮偏角
veh_params.velocity = 15;     %车速, m/s
veh_params.v_des = veh_params.velocity;       %期望车速, m/s
veh_params.angular_v=0;       %当前车辆角速度
roadmap_name = 'big_circle';  %选择地图: small_circle, big_circle
path_tracking_alg = 'LQR';

%% 2. 仿真参数
i = 0;                  %仿真时间index
time_step = 0.1;        %仿真步长, m

simulation_time = 0;    %当前仿真时间, s
run_distance = 0;       %车辆行驶里程, m

realCmd=[];              %store commend of steer

%% 3.车辆参数
% 3.1. 车辆转向控制参数
veh_params.wheel_base = 2.5;    %轴距, m
veh_params.max_steer_angle = 53 / 180 * pi; %前轮最大转角, rad /apollo 53, old 30
veh_params.max_angular_vel = 53 / 180 * pi; %前轮最大角速度, rad/s
veh_params.max_acceleration = 11; %车辆最大加速度
veh_params.max_deceleration = 12; %车辆最大减速度

% 3.2. 车的大小及长度参数
veh_params.vehicle_size = 20;
veh_params.vehicle_length = veh_params.velocity * time_step * 0.8;

%% 4. 生成期望路径及长度参数
[trajref_params, simulation_stop_y, simulation_stop_time] =...
    set_trajref_params(roadmap_name, veh_params);   %设置trajref参数
trajref = generate_trajref(trajref_params,roadmap_name);         %生成trajref

%% 5. 载入LQR参数设置
lqr_params = load_lqr_params(trajref_params,veh_params);

%% 6. 期望路径及车辆位姿可视化
[path_figure, steer_figure] = draw_path_tracking(...
    path_tracking_alg, roadmap_name, trajref, veh_pose, steer_state,...
    veh_params, simulation_time, simulation_stop_time);

%% 7. 初始化log参数
log = log_init(time_step, simulation_stop_time);

%pause;  %暂停以显示路网及车辆初始位姿

%% 8. 采用指定的路径跟踪算法使车辆沿期望路径行驶
disp([path_tracking_alg,' simulation start!']);
while((simulation_time < simulation_stop_time) &&...
        (veh_pose(2) < simulation_stop_y))
    
    tic;    %计算一个周期的时间
    
    % 8.1. 更新仿真时间
    i = i + 1;  %更新i值，用以记录相关log
    simulation_time = simulation_time + time_step;
    run_distance = run_distance + veh_params.velocity * time_step;
    log.time(i) = simulation_time;  %记录仿真时间
    log.dist(i) = run_distance;     %记录车辆仿真行驶里程
   
    % 8.2. 计算期望前轮偏角
    steer_cmd = ALG_LQR(veh_pose, trajref,...
    lqr_params, veh_params, steer_state, time_step);
    steer_cmd_new=steer_cmd;
        
    % 8.3. 更新前轮偏角及车辆位姿
    steer_state = steer_cmd_new;
    veh_pose = update_veh_pose(veh_pose, steer_state, veh_params,time_step);
    
    % 计算车辆当前位置在期望路径上的投影点位姿
    [~, index] = calc_nearest_point(veh_pose, trajref);
    ref_pose = calc_proj_pose(veh_pose(1:2), trajref(index, 1:3),...
    trajref(index + 1, 1:3));
    log.ref_pose(i, :) = ref_pose;
    log.steer_cmd(i) = steer_cmd_new / pi * 180;    %记录期望转角
    log.veh_pose(i, :) = veh_pose;            	%记录车辆位姿
    log.veh_speed(i, :) = veh_params.velocity;  %记录车辆速度
    
    % 8.3.1 更新车辆当前角速度
    veh_params.angular_v=update_angular_velocity(log,time_step,i);
    
    % 8.4. 车辆位姿及前轮偏角可视化
    set(groot, 'CurrentFigure', path_figure);   %设为当前figure
    draw_veh_pose(veh_pose, veh_params);    	%更新车辆位姿
    set(groot, 'CurrentFigure', steer_figure);
    plot(log.time(i), log.steer_cmd(i), 'b.', 'markersize', 20);
    
    % 8.5. 可视化每一个周期变化
    cycle_time = toc;   % 计算一个周期的运算时间
    log.cycle_time(i,:)=cycle_time;
    pause(0.01);        %动画显示
end

disp('simulation end!');







