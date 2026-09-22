%% a001_new_Design_LQR_Controller.m：第四章路径跟踪与分阶段内环维修
% 第4章 4.2.2 LQR路径跟踪控制器
% 唯一日常入口：车辆参数、操场几何与运行设置；在线 LQR 在模型块内求解。

% 不清空调用者工作区；本入口只初始化参数与几何表，不启动仿真。

%% =========================
% 1) 车辆参数输入（车辆沿用实测echo；接口以2026-09-21 18:52 GUI导出为准）
% =========================
m = 1843;     % 整车质量 [kg]
Iz = 3462.948099;     % 转动惯量 [kg*m^2]
a = 1.177744981;     % 前轴到质心距离 [m]
b = 1.682255019;     % 后轴到质心距离 [m]
% 当前 CarSim 轮胎表在静载、零侧偏/零纵滑附近的等效轴级刚度；不是全域 Fiala 等价。
Caf = 165585.267615143;   % 前轴侧偏刚度 [N/rad]
Car = 118001.807784225;    % 后轴侧偏刚度 [N/rad]
d = 1.675;      % 轮距 [m]
h = 0.4942892024;     % 质心高度 [m]
Rw = 0.32;     % 车轮半径 [m]
Iw = 1.5;      % 车轮转动惯量 [kg*m^2]
mu = 0.85;

% 论文方向盘等效约束除以14.6得到前轮约束；不再连接方向盘接口或齿条查表。

kw=5;       %轮速反馈增益:实际轮速比目标轮速高 1 rad/s，kw=5 就会产生 −5 N·m 的转矩修正，用来减小轮速误差。

% 左右目标纵向力按CarSim四轮实际轮荷分配；不再使用固定前后载荷转移系数。
k_beta_p = 1.5;
k_beta_d = 0.1;
k_beta = 0.1;
k_r_p = 5;
k_r_d = 0.50;
k_V = 2.0;

% k_beta_p = 1.5;
% k_beta_d = 0.1;
% k_beta = 0.1;
% k_r_p = 5;
% k_r_d = 0.50;
% k_V = 2;

%% 统一算法参数（CarSim响应不经外加物理或执行器限幅）
% 车辆参数和控制增益沿用上面的定义；执行输出不经过人为可行域门控。

% 不使用附着、轮荷、转向幅值或力矩可行域门槛；保留连续数值正则化。

% CarSim 是车辆参数来源；以下值用于追溯，不再反向覆盖 CarSim 车辆。
carsim_cfg.M_SU = 1600;
carsim_cfg.LX_CG_SU = 1170;
carsim_cfg.H_CG_SU = 520;
carsim_cfg.IZZ_SU = 2800;
carsim_cfg.track_mm = 1675;
carsim_cfg.RRE_mm = 320;
carsim_cfg.IT = 1.5;
carsim_cfg.alpha_scale = [1,1];
carsim_cfg.source_path = 'C:\Users\Public\Documents\CarSim2020.0_Data\Results\Run_54cb954f-d91a-4b4d-b546-9ec1dbf045e8\Run_all.par';
carsim_cfg.source_sha256 = '31889789adfa370a49b43523d5c6d43144469a85767a64ab199adf7da2067839';
repair_root = fileparts(mfilename('fullpath'));
% 主链已采用直接前轮角，旧齿条查表仅留在维修快照中。
Ts_plant = 0.0005;             % CarSim交换与ode4固定积分步长 [s]

%% 分阶段工况：普通直线—左转—直线；不规划漂移动作
Ts=0.01; t_final=17; R=80; Vr=10.5; beta_r=0;
% 普通工况为工程诊断：名义横向加速度Vr^2/R=1.378m/s^2。
% 40m初始直线、25m入弯、40m恒曲率、25m出弯，然后保持直线。
% mode:1入漂维持；2同步退出；3先beta回正再降曲率；4原操场；5提前回正操场；6普通转弯。
% 独立退出的exit_s只由通过3秒稳态门槛的前置试验生成，不使用猜测初值。
path_cfg.R=R;path_cfg.straight=40;path_cfg.transition=25;
path_cfg.plateau=40; % 本普通工况恒曲率弧长[m]，总转角(25+40)/80=46.55deg
path_cfg.half=path_cfg.straight+2*path_cfg.transition+path_cfg.plateau;
path_cfg.length=2*path_cfg.half;path_cfg.V=Vr;path_cfg.beta=beta_r;
path_cfg.ramp_time=2;path_cfg.care_tol=1e-8; % CARE残差仅记录，不作为仿真门槛
path_cfg.mode=6;path_cfg.exit_s=NaN; % 6=普通开口轨迹；不循环、不启用漂移分支
path_Q=diag([10,10,2]);path_R=diag([100,100,10]);
addpath(fullfile(repair_root,'repair_support'));
% 入漂诊断：侧偏过渡长度和相对曲率的延后距离；均为工程工况参数，单位m。
path_cfg.beta_transition=path_cfg.transition;path_cfg.beta_delay=0;
path_geometry=repair_build_geometry(path_cfg);
%% 原始控制接口：物理参数与控制增益统一来自以上入口
repair_cfg=struct;
repair_cfg.m=m;
repair_cfg.Iz=Iz;
repair_cfg.a=a;
repair_cfg.b=b;
repair_cfg.d=d;
repair_cfg.h=h;
repair_cfg.Rw=Rw;
repair_cfg.Iw=Iw;
repair_cfg.mu=mu;
repair_cfg.Caf=Caf;
repair_cfg.Car=Car;
repair_cfg.kw=kw;
repair_cfg.Ts=Ts;
repair_cfg.Ts_plant=Ts_plant;
repair_cfg.k_beta_p=k_beta_p;
repair_cfg.k_beta_d=k_beta_d;
repair_cfg.k_beta=k_beta;
repair_cfg.k_r_p=k_r_p;
repair_cfg.k_r_d=k_r_d;
repair_cfg.k_V=k_V;
% 仅用于平滑轮速逆解在Fy过零处的数值变化，不限制目标力或执行器输出。
repair_cfg.force_eps_frac=0.02; % 按轴载荷容量比例设置连续正则化尺度
repair_interface_test=0;             % 仅验证脚本临时置1；日常闭环不旁路控制

%% CarSim只引用用户GUI导出的现有文件
% 不调用repair_prepare_carsim，不生成或改写任何sim/par。
repair_simfile='C:\Users\Public\Documents\CarSim2020.0_Data\Extensions\Simulink\00carsim_workspace\009\simfile.sim';
assert(isfile(repair_simfile),'现有GUI sim文件不存在，请由用户在CarSim中重新生成。');
addpath('C:\Program Files (x86)\CarSim2020.0_Prog\Programs\solvers\Matlab84+');
fprintf('模型使用现有GUI配置：%s\n',repair_simfile);


%% 可选研究模式：普通直行→滑移建立→入漂→局部维持
% 在运行本入口前设置repair_run_mode='drift_entry'启用；'normal'保持原普通路径模式。
% 此入口只初始化，不启动仿真。25m/50m均通过原生对照，默认25m。
if ~exist('repair_run_mode','var'), repair_run_mode='normal'; end
assert(ismember(string(repair_run_mode),["normal","drift_entry"]),'repair_run_mode只支持normal或drift_entry');
if ~exist('repair_entry_length','var'), repair_entry_length=25; end
assert(ismember(repair_entry_length,[25,50]),'入漂长度仅有25m和50m经过本轮对照');
entry_bank_source=jsondecode(fileread(fullfile(repair_root,'repair_support','repair_entry_bank.json')));
% 参数只把数值结构送入MATLAB Function；来源、中文说明和标签留在工作区供核对。
entry_cfg=rmfield(entry_bank_source,{'labels','source_hashes','note','source_par_sha256'});
entry_cfg.length=repair_entry_length;
repair_cfg.entry_steer_rate=entry_cfg.steering_rate_rad_s;
repair_cfg.entry_mode=double(strcmp(repair_run_mode,'drift_entry'));
path_cfg.feedback_enabled=1-repair_cfg.entry_mode;
if repair_cfg.entry_mode==1
    % 控制库与当前GUI配置绑定，避免更换车辆后仍套用这一组局部增益。
    fid=fopen(carsim_cfg.source_path,'rb');assert(fid>=0);bytes=fread(fid,Inf,'*uint8');fclose(fid);
    md=java.security.MessageDigest.getInstance('SHA-256');md.update(bytes);
    digest=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
    assert(strcmp(digest,entry_bank_source.source_par_sha256),'CarSim配置已经变化，需要重新验证起漂工作点库');
    fields=fieldnames(entry_cfg.vehicle);
    for k=1:numel(fields),assert(abs(repair_cfg.(fields{k})-entry_cfg.vehicle.(fields{k}))<1e-8,'物理参数与工作点库不一致');end
    t_final=30;
    fprintf('有效运行模式：直行→入漂→维持，过渡%d m，目标12m/s、beta=-10deg、AVz=0.6rad/s；路径反馈关闭。\n',repair_entry_length);
else
    fprintf('有效运行模式：原普通路径跟踪，17s、R=80m、beta_ref=0；新增局部漂移支路不驱动车辆。\n');
end
