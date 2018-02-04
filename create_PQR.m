%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Defining the sampling time
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ts = 0.01;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%P 42 by 42
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

p_dd_x = 1;
p_dd_y = 1;
p_dd_z = 1;

vec_p_acc = [p_dd_x;p_dd_y;p_dd_z];
vec_p_vel = ts*vec_p_acc;
vec_p_pos = ts*vec_p_vel;

p_dd_phi = 1;
p_dd_theta = 1;
p_dd_psi = 1;

vec_p_ddangles = [p_dd_phi;p_dd_theta;p_dd_psi];
vec_p_dangles = ts*vec_p_ddangles;
vec_p_angles = ts*vec_p_dangles;

p_dd_theta_hip = 1;
p_dd_psi_hip = 1;
p_dd_theta_knee = 1;
p_dd_theta_ankle = 1;

vec_p_ddlegs = [p_dd_theta_hip;p_dd_psi_hip;p_dd_theta_knee;p_dd_theta_ankle];
vec_p_dlegs = ts*vec_p_ddlegs;
vec_p_legs = ts*vec_p_dlegs;

P = [...
    vec_p_pos;...           
    vec_p_angles;...        
    vec_p_legs;...          
    vec_p_legs;...
    vec_p_vel;...
    vec_p_dangles;...
    vec_p_dlegs;...
    vec_p_dlegs;...
    vec_p_acc;...
    vec_p_ddangles;...
    vec_p_ddlegs;...
    vec_p_ddlegs...
    ];

P = diag(P);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Q Matrix 42 by 42
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

q_dd_x = 125;
q_dd_y = 80;
q_dd_z = 150;

vec_q_acc = [q_dd_x;q_dd_y;q_dd_z];
vec_q_vel = ts*vec_q_acc;
vec_q_pos = ts*vec_q_vel;

q_dd_phi = 125;
q_dd_theta = 175;
q_dd_psi = 75;

vec_q_ddangles = [q_dd_phi;q_dd_theta;q_dd_psi];
vec_q_dangles = ts*vec_q_ddangles;
vec_q_angles = ts*vec_q_dangles;

q_dd_theta_hip = 55;
q_dd_psi_hip = 20;
q_dd_theta_knee = 65;
q_dd_theta_ankle = 20;

vec_q_ddlegs = [q_dd_theta_hip;q_dd_psi_hip;q_dd_theta_knee;q_dd_theta_ankle];
vec_q_dlegs = ts*vec_q_ddlegs;
vec_q_legs = ts*vec_q_dlegs;

Q = [...
    vec_q_pos;...           
    vec_q_angles;...       
    vec_q_legs;...          
    vec_q_legs;...         
    vec_q_vel;...
    vec_q_dangles;...
    vec_q_dlegs;...
    vec_q_dlegs;...
    vec_q_acc;...
    vec_q_ddangles;...
    vec_q_ddlegs;...
    vec_q_ddlegs...
    ];

Q = diag(Q)/1000;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%R Matrix input noise nxn (n = number of inputs) 16 pixel in 14 IMU
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%from the sensor
r_accelerometer = (1E-3)^2;             %3 degrees
r_gyroscope = (5*pi/180)^2;             %3 degrees
r_magnetometer = 0.25^2;                %3 degrees
r_barotometer = 5.9;                    %1 degrees

%gps
r_gps_pos = 5^2;                        %2 degrees
r_gps_vel = 0.5;                        %1 degrees
r_gps_head = (15*pi/180)^2;             %1 degrees

%from the cameras we have pixels as measurements
r_pixel = 200;                          %16 degrees  maybe 32





