% foc_params.m — Simulink FOC 仿真全局参数
%   由 foc_sim.slx / foc_controller.slx 的 InitFcn 自动调用
%   所有 PI / 限幅参数均在此统一管理，禁止在模型代码中硬编码


% -- 1. 电机本体参数 (GM3506 参考值 / 估算值) --
p     = single(11);         % 极对数 (24N/22P → 22/2=11)
Rs    = single(2.7);        % 相电阻 [Ohm] (实测线间 5.4Ω / 2，Y接法)
Ld    = single(0.2e-3);     % d轴电感 [H]
Lq    = single(0.2e-3);     % q轴电感 [H]
Psi_f = single(0.01);       % 永磁体磁链 [Wb]
J     = single(1e-5);       % 转子转动惯量 [kg*m^2]
B     = single(0.01);       % 黏性摩擦系数 [N*m*s/rad] (堵转测试用较大值)

% -- 2. 系统环境参数 --
V_bus_nom = single(12.0);   % 母线额定电压 [V]
T_load    = single(0.0);    % 外部负载转矩 [Nm]
Ts        = 50e-6;          % Simulink SampleTime [s] (必须 double, 给 solver/采样用)
Ts_ctrl   = single(Ts);     % 控制器内部用 [s] (single, 给 PI 积分计算)

% -- 3. 系统零位偏差 --
theta_offset = single(0.0); % 机械角零点偏差 [rad]

% -- 4. 电流环 PI 参数（极点配置法, 带宽 2kHz）--
f_bw  = 500;                           % 电流环带宽 [Hz]（先保守，稳定后再提）
wc    = 2*pi*f_bw;
Kp_d  = single(Ld * wc);               % d轴 Kp = Ld * wc ≈ 2.51
Ki_d  = single(Rs * wc);               % d轴 Ki = Rs * wc ≈ 3770
Kp_q  = single(Lq * wc);               % q轴 Kp
Ki_q  = single(Rs * wc);               % q轴 Ki
id_ref = single(0.0);                  % d轴电流参考 [A] (SPM: id=0)
iq_ref = single(0.2);                  % q轴电流参考 [A]

% -- 5. SVPWM / 输出限幅 --
V_max = single(V_bus_nom / sqrt(single(3)));  % 线性调制区电压上限 [V]
duty_min = single(0.02);               % 占空比下限
duty_max = single(0.98);               % 占空比上限

