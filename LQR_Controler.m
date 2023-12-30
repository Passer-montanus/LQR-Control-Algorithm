% ·�����ٺ���LQR����������

clc
clear
close all
addpath  Calc_fcns Draw_fcns Limited&Update %���뺯���ļ���
%% 1. ���ó�����ʼ״̬
veh_pose = [0, -15, 0];       %������ʼλ��
steer_state = 0;            %��ǰʱ��ǰ��ƫ��
veh_params.velocity = 15;     %����, m/s
veh_params.v_des = veh_params.velocity;       %��������, m/s
veh_params.angular_v=0;       %��ǰ�������ٶ�
roadmap_name = 'big_circle';  %ѡ���ͼ: small_circle, big_circle
path_tracking_alg = 'LQR';

%% 2. �������
i = 0;                  %����ʱ��index
time_step = 0.1;        %���沽��, m

simulation_time = 0;    %��ǰ����ʱ��, s
run_distance = 0;       %������ʻ���, m

realCmd=[];              %store commend of steer

%% 3.��������
% 3.1. ����ת����Ʋ���
veh_params.wheel_base = 2.5;    %���, m
veh_params.max_steer_angle = 53 / 180 * pi; %ǰ�����ת��, rad /apollo 53, old 30
veh_params.max_angular_vel = 53 / 180 * pi; %ǰ�������ٶ�, rad/s
veh_params.max_acceleration = 11; %���������ٶ�
veh_params.max_deceleration = 12; %���������ٶ�

% 3.2. ���Ĵ�С�����Ȳ���
veh_params.vehicle_size = 20;
veh_params.vehicle_length = veh_params.velocity * time_step * 0.8;

%% 4. ��������·�������Ȳ���
[trajref_params, simulation_stop_y, simulation_stop_time] =...
    set_trajref_params(roadmap_name, veh_params);   %����trajref����
trajref = generate_trajref(trajref_params,roadmap_name);         %����trajref

%% 5. ����LQR��������
lqr_params = load_lqr_params(trajref_params,veh_params);

%% 6. ����·��������λ�˿��ӻ�
[path_figure, steer_figure] = draw_path_tracking(...
    path_tracking_alg, roadmap_name, trajref, veh_pose, steer_state,...
    veh_params, simulation_time, simulation_stop_time);

%% 7. ��ʼ��log����
log = log_init(time_step, simulation_stop_time);

%pause;  %��ͣ����ʾ·����������ʼλ��

%% 8. ����ָ����·�������㷨ʹ����������·����ʻ
disp([path_tracking_alg,' simulation start!']);
while((simulation_time < simulation_stop_time) &&...
        (veh_pose(2) < simulation_stop_y))
    
    tic;    %����һ�����ڵ�ʱ��
    
    % 8.1. ���·���ʱ��
    i = i + 1;  %����iֵ�����Լ�¼���log
    simulation_time = simulation_time + time_step;
    run_distance = run_distance + veh_params.velocity * time_step;
    log.time(i) = simulation_time;  %��¼����ʱ��
    log.dist(i) = run_distance;     %��¼����������ʻ���
   
    % 8.2. ��������ǰ��ƫ��
    steer_cmd = ALG_LQR(veh_pose, trajref,...
    lqr_params, veh_params, steer_state, time_step);
    steer_cmd_new=steer_cmd;
        
    % 8.3. ����ǰ��ƫ�Ǽ�����λ��
    steer_state = steer_cmd_new;
    veh_pose = update_veh_pose(veh_pose, steer_state, veh_params,time_step);
    
    % ���㳵����ǰλ��������·���ϵ�ͶӰ��λ��
    [~, index] = calc_nearest_point(veh_pose, trajref);
    ref_pose = calc_proj_pose(veh_pose(1:2), trajref(index, 1:3),...
    trajref(index + 1, 1:3));
    log.ref_pose(i, :) = ref_pose;
    log.steer_cmd(i) = steer_cmd_new / pi * 180;    %��¼����ת�Ǭ角
    log.veh_pose(i, :) = veh_pose;            	%��¼����λ��
    log.veh_speed(i, :) = veh_params.velocity;  %��¼�����ٶ�
    
    % 8.3.1 ���³�����ǰ���ٶ�
    veh_params.angular_v=update_angular_velocity(log,time_step,i);
    
    % 8.4. ����λ�˼�ǰ��ƫ�ǿ��ӻ�
    set(groot, 'CurrentFigure', path_figure);   %��Ϊ��ǰfigure
    draw_veh_pose(veh_pose, veh_params);    	%���³���λ��
    set(groot, 'CurrentFigure', steer_figure);
    plot(log.time(i), log.steer_cmd(i), 'b.', 'markersize', 20);
    
    % 8.5. ���ӻ�ÿһ�����ڱ仯
    cycle_time = toc;   % ����һ�����ڵ�����ʱ��
    log.cycle_time(i,:)=cycle_time;
    pause(0.01);        %������ʾ
end

disp('simulation end!');







