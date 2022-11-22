%% Code Manipulator 2DOF
% Clean variables
clc, clear all, close all;

% Time defintion variables
t_s = 0.01;
t_final = 100;
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
q = zeros(4, length(t)+1);
q(:, 1) = [170*pi/180;...
           -80*pi/180;...
           0;...
           0];
       

% Constant defintion
constans = [g, t_s];

% Robot definition
robot = manipulator_system(L1, L2, constans, q(:,1));

% Inital Position Carterian Space
x = zeros(2, length(t)+1);
xp = zeros(2, length(t)+1);
x(:, 1) = robot.get_general_position();
xp(:, 1) = robot.get_general_velocities();

% Desired angles of the system
qd = [90*pi/180*ones(1, length(t));...
       0*pi/180*ones(1, length(t))];
   
qdp = [0*pi/180*ones(1, length(t));...
       0*pi/180*ones(1, length(t))];
   
qdpp = [0*pi/180*ones(1, length(t));...
        0*pi/180*ones(1, length(t))];

% Desired Position cartesian Space
xd_c = [0.3*sin(0.1*t);...
      0.1*ones(1, length(t))];
  
xdp_c = [(0.3*0.1)*cos(0.1*t);...
       0*ones(1, length(t))];

xdpp_c = [-(0.3*0.1*0.1)*sin(0.1*t);...
        0*ones(1, length(t))];

% Control gains
kp = 2;
wn = sqrt(kp);
kv = 14*1*wn;

ki = 20;
kp_force = 2;
wn_force = sqrt(kp_force);
kv_force = 20*1*wn_force;
% Learning variable
max_value = (((kv^2)/2)-2)/(sqrt((kv^2)/(4)-1));

gamma = 0.1;

% Learning system
% Transfer funtion definitions
P = tf([1 (kv-gamma) kp-gamma], [0 0 1]);
A = tf([1], [1 kv kp]);
S = tf([0 0 1], [0 1 0]);

% Filter force design
kp_f = 1;
wn_f = sqrt(kp_f);
kv_f = 1.2*1*wn_f;

Filter = tf([1], [1 kv_f kp_f]);
Filter_d = c2d(Filter, t_s);
[num1d_filter, den1d_filter] = tfdata(Filter_d,'v');

% Filter coeficients
A_filter = num1d_filter(2:end);
B_filter = den1d_filter(2:end);

% Operator contraction
aux = S*P*A;

% Operator Function
aux_d = c2d(aux, t_s);
[num1d, den1d] = tfdata(aux_d,'v');


% PD control Gains
K1 = kp*eye(2);
K2 = kv*eye(2);
Km = ki*eye(2);

KP_force = kp_force*eye(2);
KD_force = kv_force*eye(2);

% Controller definition
control = controller(K1, K2, Km, KP_force, KD_force, num1d, den1d, robot);

% Control vector empty
u = zeros(2, length(t));
u_cartesian = zeros(2, length(t));

% Control vector error definition
qe = zeros(2, length(t));
qep = zeros(2, length(t));

xe = zeros(2, length(t));
xep = zeros(2, length(t));

% External torque of the system
T_extern = zeros(2, length(t));

% External Force Reaction
x_enviroment = [1.2*ones(1, length(t)+1);...
                0 + 0.5*sin(0.1*[t, t(end)+ t_s])];
            
x_enviroment_c = (-0.5:0.01:0.5);
x_enviroment_c(2, :) = 0*ones(1, length(x_enviroment_c));

% Location New Axis
r_c = [1.2;0];
% Angle new Axis
angle_c = 45*pi/180;

Rot_c = [cos(angle_c), -sin(angle_c);...
         sin(angle_c), cos(angle_c)];
     
x_enviroment_i = r_c + Rot_c*x_enviroment_c;     
% Enviroment Force
F_enviroment = zeros(2, length(t)+1);


% Potential Field
[V, index] = potential_field(x(:, 1), x_enviroment_i(:, :));

% Angle Between end effector and Obstacle
beta = angle_obstacle(x(:, 1), x_enviroment_i(:, index));

F_enviroment(:, 1) = [cos(beta);sin(beta)]*V;

% Forces Frame C
F_c(:,1) = inv(Rot_c)*F_enviroment(:,1);

% Force Filter
F_x_memory = zeros(length(den1d_filter(2:end)),1);
F_y_memory = zeros(length(den1d_filter(2:end)),1);

F_x_filter =  zeros(length(num1d_filter(2:end)),1);
F_y_filter =  zeros(length(num1d_filter(2:end)),1);

% Deisred Trayectory I frame
xd_i = r_c + Rot_c*xd_c;
xdp_i = Rot_c*xdp_c;
xdpp_i = Rot_c*xdpp_c;

% Desired Forces
F_desired = [0*ones(1, length(t)+1);...
             4*ones(1, length(t)+1)];
         
F_desired_p = [0*ones(1, length(t)+1);...
               0*ones(1, length(t)+1)];
           
F_desired_pp = [0*ones(1, length(t)+1);...
                0*ones(1, length(t)+1)];

% Derivative of the Force
Force_c_filter_p = zeros(2, length(t));

for k = 1:length(t)
    % Filter Force 
    [F_x_filter_k, F_x_filter, F_x_memory] = filter(F_enviroment(1, k), F_x_filter, F_x_memory, A_filter, B_filter);
    [F_y_filter_k, F_y_filter, F_y_memory] = filter(F_enviroment(2, k), F_y_filter, F_y_memory, A_filter, B_filter);
    
    % Force I frame filter
    Force_filter(:, k) = [F_x_filter_k;...
                    F_y_filter_k];
    % Force C frame Filter 
    Force_c_filter(:, k) = inv(Rot_c)*Force_filter(:, k);
    
    % Derivative Force 
    if k > 1
        Force_c_filter_p(:, k) = (Force_c_filter(:, k) - Force_c_filter(:, k-1))/t_s;
    end
    % Control vector
    qe(:, k) = qd(:, k) - robot.get_positions();
    qep(:, k) = qdp(:, k) - robot.get_velocities();
    
    %Position Cartesian Space
    xe(:, k) = xd_c(:, k) - inv(Rot_c)*(robot.get_general_position()-r_c);
    xep(:, k) = xdp_c(:, k) - inv(Rot_c)*(robot.get_general_velocities());
    
    % Position Frame C
    x_c(:, k) = inv(Rot_c)*(robot.get_general_position()-r_c);
    % Force Control Hibrid Mode
    fe(:, k) = F_desired(:, k) - Force_c_filter(:, k);
    
    % Control  robot
    %u(:, k) = control.get_control_PD_Gravity(qd(:, k), qdp(:, k));
    u_cartesian(:, k) = control.get_control_hibrid(xd_c(:, k), xdp_c(:, k), xdpp_c(:, k), Force_filter(:, k), Rot_c, r_c, Force_c_filter(:, k), Force_c_filter_p(:, k), F_desired(:, k), F_desired_p(:, k), F_desired_pp(:, k));
    
    % System evolution
    q(:, k+1) = robot.system_f(u_cartesian(:, k), Force_filter(:, k));
    x(:, k+1) = robot.get_general_position();
    xp(:, k+1) = robot.get_general_velocities();
    
    % Potential Field
    [V(:, k+1), index] = potential_field(x(:, k+1), x_enviroment_i(:, :));
    
    %Angle To Obstacle genral Coordinate
    beta(:, k+1) = angle_obstacle(x(:, k+1), x_enviroment_i(:, index));
    
    %External Force
    F_enviroment(:, k+1) = [cos(beta(:, k+1));sin(beta(:, k+1))]*V(:, k+1);
    
    %External Force frame C
    F_c(:, k+1) = inv(Rot_c)*F_enviroment(:, k+1);
end

for k = 1:10:length(t)
    drawpend2(q(:, k), m1, m2, 0.3, l1, l2, x_enviroment_i(:, :), xd_i(:,k));
end

save("Data_Hibrid.mat", "t", "q", "qd", "qdp", "qe", "qep", "u_cartesian", "T_extern","L1", "L2", "xd_c", "xdp_c", "x", "xp", "xe", "xep", "F_enviroment", "V", "beta", "xd_i", "xdp_i", "F_c", "Force_filter", "Force_c_filter", "F_desired", "fe", "x_c", "x_enviroment_i")