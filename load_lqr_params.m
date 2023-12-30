function lqr_params = load_lqr_params(trajref_params,...
    veh_params)
% ����LQR�Ĳ���

% ����:
% trajref_params : trajref�Ĳ���
% veh_params     : veh�Ĳ���

% set up vehicle parameters
[cf, cr, mass, lf, lr, iz] = set_veh(veh_params);

% set up matrix dimension
 basic_state_size = 4;
 control_size = 1;
 
% setup and save the Matirx and paramters
lqr_params = set_matrix(basic_state_size, cf, cr, mass, lf, lr, iz, control_size, trajref_params, veh_params);


matrix_q=zeros(4,4);                  %����Ȩ�ؾ������
matrix_q(1,1)=0.05;
matrix_q(3,3)=1;
lqr_params.Q =matrix_q ;     %״̬���Ȩ�ؾ���

lqr_params.R = eye(1,1);      %������Ȩ�ؾ���

lqr_params.Q0 = eye(4,4);     %״̬����ն˾���


function [cf, cr, mass, lf, lr, iz] = set_veh(veh_params)
cf = 155494.663;        %ǰ�ֲ�ƫ�ն�(cornering stiffness)  
cr = 155494.663;        %���ֲ�ƫ�ն�

mass_fl = 520;           %ǰ������
mass_fr = 520;
mass_rl = 520;
mass_rr = 520;

mass_front =  mass_fl + mass_fr;
mass_rear =  mass_rl + mass_rr;
mass = mass_front + mass_rear;

lf = veh_params.wheel_base * (1.0 - mass_front / mass); %ǰ������
lr = veh_params.wheel_base * (1.0 - mass_rear / mass);  %��������
iz = lf * lf * mass_front + lr * lr * mass_rear;  %������z��ת����ת������

function lqr_params = set_matrix(basic_state_size, cf, cr, mass, lf, lr, iz, control_size, trajref_params, veh_params)
% matrix a
matrix_a=zeros(basic_state_size,basic_state_size);
matrix_a_coeff=zeros(basic_state_size,basic_state_size);
I=eye(basic_state_size,basic_state_size);

matrix_a(1,2) = 1.0;
matrix_a(2,3) = (cf + cr ) / mass;
matrix_a(3,4) = 1.0;
matrix_a(4,3) = (lf * cf - lr * cr) / iz;

matrix_a_coeff(2,2) = -(cf + cr) / mass;
matrix_a_coeff(2,4) = (lr * cr - lf * cf) / mass;
matrix_a_coeff(3,4) = 1.0;
matrix_a_coeff(4,2) = (lr * cr - lf * cf) / iz;
matrix_a_coeff(4,4) = -1.0 * (lf * lf * cf + lr * lr * cr)/iz;

matrix_a(2,2)=matrix_a_coeff(2,2);
matrix_a(2,4)=matrix_a_coeff(2,3);
matrix_a(4,2)=matrix_a_coeff(4,2);
matrix_a(4,4)=matrix_a_coeff(4,4);


%matrix b
matrix_b=zeros(basic_state_size,control_size);
matrix_b(2,1) = cf/mass;
matrix_b(4,1) = lf * cf /iz;


% save the Matirx and paramters
lqr_params.A=matrix_a;
lqr_params.B=matrix_b;
lqr_params.I=I;
lqr_params.ts = 0.01; % time period
lqr_params.lr=lr;
lqr_params.lf=lf;
lqr_params.cf=cf;
lqr_params.cr=cr;
lqr_params.iz=iz;
lqr_params.mass=mass;

lqr_params.delta_t = trajref_params.dist_interval /...
    veh_params.velocity;              %LQR��ʱ�䲽��, s

lqr_params.ref_index = 0;      %���ٲο���
lqr_params.horizon = 15;       %�����Ż����ڵĴ�С
