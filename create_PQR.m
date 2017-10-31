%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%P 42 by 42
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

P = diag(0.001*ones(42,1));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Q Matrix 42 by 42
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ts = 0.01;

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
    vec_q_pos;...           %body pos
    vec_q_angles;...        %body angs
    vec_q_legs;...          %left leg
    vec_q_legs;...          %right leg
    
    vec_q_vel;...
    vec_q_dangles;...
    vec_q_dlegs;...
    vec_q_dlegs;...
    
    vec_q_acc;...
    vec_q_ddangles;...
    vec_q_ddlegs;...
    vec_q_ddlegs...
    ];

Q = diag(Q);

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
r_pixel = 5^2;                          %16 degrees  maybe 32









