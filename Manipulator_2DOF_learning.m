%% Code Manipulator 2DOF
% Clean variables
clc, clear all, close all;

% Time defintion variables
t_s = 0.01;
t_final = 80;
t = (0:t_s:t_final);

g = 9.8;
% System parameters L1
b1 = 1;
m1 = 0.8;
l1 = 1;
Iz1= 0.5;

% System parameters L2
b2 = 1;
m2 = 0.8;
l2 = 1;
Iz2= 0.5;


L1 = [b1 , m1, l1, Iz1];
L2 = [b2 , m2, l2, Iz2];

% Initial conditions system       
q = [];
q(:, 1) = [0*pi/180;...
           0;...
           0;...
           0];

% Constant defintion
constans = [g, t_s];

% Robot definition
robot = manipulator_system(L1, L2, constans, q(:,1));


% Desired angles of the system
qd = [90*pi/180 + 10*pi/180*sin(0.5*t);...
       0*pi/180 + 10*pi/180*sin(0.5*t)];

qdp = [0.5*(10*pi/180)*cos(0.5*t);...
       0.5*(10*pi/180)*cos(0.5*t)];
   
qdpp = [-(0.5)^2*(10*pi/180)*sin(0.5*t);...
        -(0.5)^2*(10*pi/180)*sin(0.5*t)];

% Aux Hd
Hd = [];

% Control gains
kp = 20;
wn = sqrt(kp);
kv = 2*1*wn;

% Learning variable
max_value = (((kv^2)/2)-2)/(sqrt((kv^2)/(4)-1));

gamma = 8.71;

% Learning system
% Transfer funtion definitions
P = tf([1 (kv-gamma) kp-gamma], [0 0 1]);
A = tf([1], [1 kv kp]);
S = tf([0 0 1], [0 1 0]);

P_operator = P*A;
% Operator contraction
aux = S*P*A;

% Operator Function
aux_d = c2d(aux, t_s);
P_operator =c2d(P_operator, t_s);
[num1d, den1d] = tfdata(aux_d,'v');


% PD control Gains
K1 = kp*eye(2);
K2 = kv*eye(2);

% Controller definition
control = controller(K1, K2, num1d, den1d, robot);

% Control vector empty
u = [];
l = [];
nominal = [];

learning_1 = zeros(1, length(t));
learning_2 = zeros(1, length(t));

% Control vector error definition
qe = [];

qep = [];

% External torque of the system
T_extern = zeros(2, length(t));

% Aux time variable 
t_aux = (t >= 10) & (t < 40);
T_extern(1, t_aux) = 10;
T_extern(2, t_aux) = 10;

% Auxiliar time definition
aux_t = 0;
t_aux = [aux_t];


for k = 1:length(t)
    % Desired vector
    Hd = [Hd, qd(:,k)];
    
    % Control vector
    qe = [qe, qd(:, k) - robot.get_positions()];
    qep = [qep, qdp(:, k) - robot.get_velocities()];
    
    % error vector Learning
    qe_k(:, k) = qd(:, k) - robot.get_positions();
 
    % Controller learning
    nominal = [nominal, control.get_control_inverse_full(qd(:, k), qdp(:, k), qdpp(:, k))];
    
    
    u = [u, control.get_control_inverse_full(qd(:, k), qdp(:, k), qdpp(:, k)) + robot.M_matrix()*control.learning_control(qd(:, k))];
    control_values(:, k) = control.get_control_inverse_full(qd(:, k), qdp(:, k), qdpp(:, k)) + robot.M_matrix()*control.learning_control(qd(:, k));
    
    
    % System evolution
    q = [q, robot.system_f(control_values(:, k), T_extern(:, k))];
    %q(:, k+1) = robot.system_f(u(:, k), T_extern(:, k));
    
    % Time auxiliar variable
    aux_t = aux_t + t_s;
    t_aux = [t_aux,aux_t];
    
end
for k = 1:10:length(t_aux)
    drawpend2(q(:, k), m1, m2, 0.3, l1, l2);
end

save("Data_Learning.mat", "t", "q", "qd", "qdp", "qe", "qep", "u", "T_extern","L1", "L2", "t_aux", "Hd")