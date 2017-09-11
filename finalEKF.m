function [x_n, P_n, x_predicted, y_err] = finalEKF(x_0, u, z, n, sAvail)
%Inputs:    x_0: estimated states at n = 0
%           u: system inputs (2/3 gyros)
%           z: measurements [Acc(3), GPS*(2), GPSV*(1), Baro(1), Mag(1),
%           P1*(3), P2*(3), P3*(3)]
%           n: number of samples for filter
%           lengths: lengths of 3 rigid beams
%           sAvail: bit string defining availability of measurements (*
%           indicates not always available)
%
%Outputs:   x_n: kalman state estimates
%           P_n: error covariance matrix
 
%Proposed states: [1: ph0, 2: th0, 3: ps0, 4: th1, 5: ps1, 6: th2, 
%                   7: ps2, 8: th3, 9:ps3, 
%                   10: Px0, 11: Py0, 12: Pz0,
%                   13: Vx0, 14: Vy0, 15: Vz0, 16: Amp1, 17: Freq1]
%       ph0, th0, ps0: roll, pitch, yaw on collar (phone gyro)
%       th2, ps2, th3, ps3: pitch and yaw from points 2 and 3 (middle and
%       tip metawears)
%       Px0, Py0: collar latitude and longitude (GPS)
%       Pz0: collar altitude (barometer)
%       bear: bearing of collar (mag)
%       Vx0, Vy0: Collar velocity is lat and long (R(bear)*integral(acc))
%       Px1, Py1, Pz1: Position of P1 (frequency and amplitude states)
%       Pa1, Pf1: amplitude and frequency states for P1 motion
 
%x_n = zeros(17, n);
x_n = [];
P_n = [];
x_predicted = [];
y_err = [];
 
l1 = 400;
l2 = 300;
l3 = 200;
 
%%%%%%%%Sensor covariances%%%%%%%%%%%%
%Making these up based on expected "noise"
GPS_lat_var = 1e-8;
GPS_long_var = 1e-8;
GPS_vel_var = 2.3;
GPSbear_var = 4.2;
P1_x_var = 35^2; %mm
P1_y_var = 20^2;
P1_z_var = 20^2;
P2_x_var = 35^2;
P2_y_var = 20^2;
P2_z_var = 20^2;
P3_x_var = 35^2;
P3_y_var = 20^2;
P3_z_var = 20^2;
 
%actual var^2 values
Acc_x_var = 2e-12;
Acc_y_var = 9e-13;
Acc_z_var = 4e-12;
Baro_var = 5.9e-5;
Mag_x_var = 0.0398;
Mag_y_var = 0.0145;
Mag_z_var = 0.2577;
 
 
%%%%%%%%%%Process Covariance%%%%%%%%%%%%%%
%from gyro static var^2 values
ph0_var = 0.05^2;
th0_var = 0.05^2;
ps0_var = 0.3^2;
% th1_var = 5e-6;
% ps1_var = 5e-6;
% th2_var = 1e-10;
% ps2_var = 8.2e-9;
th1_var = 0.01^2;
ps1_var = 0.01^2;
th2_var = 0.03^2;
ps2_var = 0.03^2;
th3_var = 0.1^2;
ps3_var = 0.1^2;
%making it up from here
Px0_var = 1e-10;
Py0_var = 1e-10;
Pz0_var = 1e-10;
Vx0_var = 1;
Vy0_var = 1;
Vz0_var = 1;
Amp1_var = 1e-5;
Freq1_var = 1e-5;
 
 
delta_t = 1/100;
x_prev = x_0;
t = ones(17, 1);
t(1) = 10;
t(2) = 10;
t(3) = 10;
t(4) = 1;
t(5) = 1;
t(6) = 2;
t(7) = 2;
t(8) = 4;
t(9) = 4;
t(16) = 10;
t(17) = 100;
P_prev = diag(t);
Q= [ph0_var, th0_var, ps0_var, th1_var, ps1_var, th2_var, ps2_var,...
th3_var, ps3_var, Px0_var, Py0_var, Pz0_var, Vx0_var, Vy0_var,...
Vz0_var, Amp1_var, Freq1_var];
 
 
for i = 1:n
    i
    %Rotate inputs u
    u_rotated = rotate_gyros(u(i, 1:3), x_prev(1:3));
    %u(i, 4:5)
    u_rotated = [u_rotated; rotate_gyros([u(i, 4:6)], [0, x_prev(4:5)])];
    temp = rotate_gyros([u(i, 7:9)], [-0.7854, 0, 0]);
    u_rotated = [u_rotated; rotate_gyros(temp, [0, x_prev(6:7)])];
    
    
    %%%%%%%%%%PREDICT%%%%%%%%%%%%%%
    
    stateCell = num2cell(x_prev);
    inputCell = num2cell(u_rotated);
    
    %x_n = f(x_n-1, u_n)
    [F, x_hat] = generated_fx(stateCell{:}, inputCell{:}, delta_t, i);
    x_predicted = [x_predicted; x_hat'];
    P_hat = F*P_prev*F' + diag(Q);
    
    %assign values to variables for easy use
    x_cell = num2cell(x_hat);
    [ph0, th0, ps0, th1, ps1, th2, ps2, th3, ps3, Px0, Py0, Pz0, Vx0, Vy0, Vz0,...
 Amp1, Freq1] = deal(x_cell{:});
    %%%%%%%%%%UPDATE%%%%%%%%%%%%%%%
    %Measurements vector (if all available):
    %[GPSx, GPSy, GPSv, P1x, P1y, P1z, P2x, P2y, P2z, P3x, P3y, P3z,
    %Accx, Accy, Accz, Baro, Magx, Magy,
    %Magz]
    
    %assign measurements to variables for easy use
    temp_measurements = num2cell(z(i,:));
    [GPSx, GPSy, GPSv, P1x, P1y, P1z, P2x, P2y, P2z, P3x, P3y, P3z,...
    Accx, Accy, Accz, Baro, Magx, Magy, Magz, GPSbear] = deal(temp_measurements{:});
    
    z_use = [];
    hx_use = [];
    H_use = [];
    R =[];
    [H, hx] = generated_hx(ph0, th0, ps0, th1, ps1, th2, ps2, th3, ...
        ps3, Px0, Py0, Pz0, Vx0, Vy0, Vz0, Amp1, Freq1,...
        x_prev(13), x_prev(14), x_prev(15), delta_t, l1, l2, l3);
    %Is GPS position available
    if(sAvail(i, 1))
        z_use = [z_use; GPSx; GPSy];
        hx_use = [hx_use; hx(1:2)];
        H_use = [H_use; H(1:2, :)];
        R = [R, GPS_lat_var, GPS_long_var];
    end
    %Is GPS velocity available
    if(sAvail(i, 1))
        z_use = [z_use; GPSv];
        hx_use = [hx_use; hx(3)];
        H_use = [H_use; H(3, :)];
        R = [R, GPS_vel_var];
    end
    %is P1 available
    if(sAvail(i, 2))
        z_use = [z_use; P1x; P1y; P1z];
       hx_use = [hx_use; hx(4:6)];
        H_use = [H_use; H(4:6, :)];
        R = [R, P1_x_var, P1_y_var, P1_z_var];
    end
    %is P2 available
    if(sAvail(i, 3))
        z_use = [z_use; P2x; P2y; P2z];
        hx_use = [hx_use; hx(7:9)];
        H_use = [H_use; H(7:9, :)];
        R = [R, P2_x_var, P2_y_var, P2_z_var];
    end
    if(sAvail(i, 4))
        z_use = [z_use; P3x; P3y; P3z];
        hx_use = [hx_use; hx(10:12)];
        H_use = [H_use; H(10:12, :)];
        R = [R, P3_x_var, P3_y_var, P3_z_var];
    end
    if(sAvail(i, 1))
        temp = sind(z(i,5)-360);
        z_use = [z_use; temp];
        hx_use = [hx_use; hx(13)];
        H_use = [H_use; H(13, :)];
        R = [R, GPSbear_var];
    end
    
    %Add the rest of the always available measurements
    z_use = [z_use; z(i, 14:20)'];
    R = [R, Acc_x_var, Acc_y_var, Acc_z_var, Baro_var, Mag_x_var, ...
        Mag_y_var, Mag_z_var];
    hx_use = [hx_use; hx(14:20)];
    H_use = [H_use; H(14:20, :)];  
    
    y = z_use - hx_use;
    y_err = [y_err; y' zeros(1, 20-size(y,1))];
    S = H_use*P_hat*H_use' + diag(R);
    K = P_hat*H_use'*inv(S);
    x_prev = (x_hat + K*y).';
    x_n = [x_n; x_prev]; 
    P_prev = (eye(17) - K*H_use)*P_hat;
    P_n = [P_n ;P_prev];
end
 
 
 
end

