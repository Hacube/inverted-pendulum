clc; clear; close all

%% system constants using parameter_sweep
m_wheel = 0.35;                   % kg - mass can be adjusted w/ bolts + print settings 
m_length = 0.048;                 % kg - mass of only the memember L
m_motor = 0.11;                   % kg
m_pend = m_motor + m_length;      % kg - mass of memeber L + motor on top
L = 0.125;                        % m
r_wheel = 0.1;                    % m
wt = 0.01;                        % m - wheel thickness
radius_motor = 0.01;              % m
g = 9.81;                         % m/s^2

% motor parameters from data sheet & motor label
rated_voltage = 12;                          % V
rated_speed = 120 * 2*pi / 60;               % rad/s
rated_torque = 0.85 * 0.09806;               % N.m
stall_torque = 4.2 * 0.09806;                % N.m
I_stall = 1;                                 % A

% mass moment of inertia for system
Im = 2/5*m_motor*radius_motor^2 + m_motor*(L+radius_motor)^2;  % motor
Is = 1/3*m_length*L^2 + Im;                                    % pendulum
Ir = 1/2*m_wheel*(r_wheel^2 + (r_wheel- wt)^2);                % wheel   

% calculate motor constants
R = rated_voltage/ I_stall;                  % ohms - motor resistance
Kt = stall_torque / rated_voltage;           % torque constant
Kv = rated_speed / rated_voltage;            % RPM/V constant

 %% matrices
        A = [0, 1, 0; m_pend*g*L/Is, 0, Kt^2/(R*Is); 0, 0, -Kt^2/(R*Ir)];
        B = [0; -Kt/(R*Is); Kt/(R*Ir)];
        C = [1, 0, 0];
        D = 0;

%% system

sys = ss(A, B, C, D);
x0 = [0; 0; 0];

%% controller

Q = eye(3);
R_control = 0.25;
K_lqr = lqr(sys,Q,R_control);


%% optimization

simout = sim('PolySim_IPRW_simulink.slx',10);

i = 0;
while simout.voltage(length(simout.voltage)) < 5
    i = i + 1;
    x0 = [deg2rad(i); 0; 0];
    simout = sim('PolySim_IPRW_simulink.slx',10);
end
display(i)