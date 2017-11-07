%%=================================================================
%%admin
%%=================================================================

clear 
close all
clc

%%=================================================================
%%preprocess data, derive eqautions, make PQR
%%=================================================================

create_PQR;             %create important matrices and initialise R values
create_data;            %create the measurement vectors   DONE    
%create_equations;       %create the measurement equations DONE

states = zeros(42,1);

mx = -0.1316;
my = 17.3801;
mz = -21.101;

%%=================================================================
%%FILTER
%%=================================================================

N = 1800;
I = eye(42);


x_estimated_store = zeros(42,N);
x_actual_store = zeros(42,N);

Pcov_store = zeros(42,42,N);
p_store = zeros(42,42,N);
f_store = zeros(42,42,N);




for i=1:1:N
    %%=================================================================
    %%Predict
    %%=================================================================
    
    zk = [];
    H = [];
    h = [];
    R = [];
    
    states = num2cell(states);
    [bodyX, bodyY, bodyZ, bodyRoll, bodyPitch, bodyYaw, LHipPitch, LHipYaw, LKneePitch, LAnklePitch,RHipPitch, RHipYaw, RKneePitch, RAnklePitch,d_bodyX, d_bodyY, d_bodyZ, d_bodyRoll, d_bodyPitch, d_bodyYaw,d_LHipPitch, d_LHipYaw, d_LKneePitch, d_LAnklePitch,d_RHipPitch, d_RHipYaw, d_RKneePitch, d_RAnklePitch,dd_bodyX, dd_bodyY, dd_bodyZ, dd_bodyRoll, dd_bodyPitch, dd_bodyYaw,dd_LHipPitch, dd_LHipYaw, dd_LKneePitch, dd_LAnklePitch,dd_RHipPitch, dd_RHipYaw, dd_RKneePitch, dd_RAnklePitch] = deal(states{:});
    states = state_predict_function(bodyX, bodyY, bodyZ, bodyRoll, bodyPitch, bodyYaw, LHipPitch, LHipYaw, LKneePitch, LAnklePitch,RHipPitch, RHipYaw, RKneePitch, RAnklePitch,d_bodyX, d_bodyY, d_bodyZ, d_bodyRoll, d_bodyPitch, d_bodyYaw,d_LHipPitch, d_LHipYaw, d_LKneePitch, d_LAnklePitch,d_RHipPitch, d_RHipYaw, d_RKneePitch, d_RAnklePitch,dd_bodyX, dd_bodyY, dd_bodyZ, dd_bodyRoll, dd_bodyPitch, dd_bodyYaw,dd_LHipPitch, dd_LHipYaw, dd_LKneePitch, dd_LAnklePitch,dd_RHipPitch, dd_RHipYaw, dd_RKneePitch, dd_RAnklePitch);
    x_estimated_store(:,i) = states;    
    
    %determining the F matrix
    Fmatrix  = Fmatrix_function(bodyX, bodyY, bodyZ, bodyRoll, bodyPitch, bodyYaw, LHipPitch, LHipYaw, LKneePitch, LAnklePitch,RHipPitch, RHipYaw, RKneePitch, RAnklePitch,d_bodyX, d_bodyY, d_bodyZ, d_bodyRoll, d_bodyPitch, d_bodyYaw,d_LHipPitch, d_LHipYaw, d_LKneePitch, d_LAnklePitch,d_RHipPitch, d_RHipYaw, d_RKneePitch, d_RAnklePitch,dd_bodyX, dd_bodyY, dd_bodyZ, dd_bodyRoll, dd_bodyPitch, dd_bodyYaw,dd_LHipPitch, dd_LHipYaw, dd_LKneePitch, dd_LAnklePitch,dd_RHipPitch, dd_RHipYaw, dd_RKneePitch, dd_RAnklePitch);
    f_store(:,:,i) = Fmatrix;
    
    %determining the P matrix
    P = (Fmatrix)*(P*(Fmatrix')) + Q;
    p_store(:,:,i) = P;
    
    
    %%=================================================================
    %%Update
    %%=================================================================
    
    %%=================================================================
    %%acceleromteter gyroscope, magnetometer
    %%=================================================================
    
    if z01Avail(i) == 1
        z01 = [body_accel_x(i);body_accel_y(i);body_accel_z(i);body_gyro_x(i);body_gyro_y(i);body_gyro_z(i);body_mag_x(i);body_mag_y(i);body_mag_z(i)];
        zk = [zk; z01];
        
        H1 = H1_matrix(dd_bodyX,dd_bodyY,dd_bodyZ,d_bodyRoll, d_bodyPitch, d_bodyYaw,mx,my,mz);
        h1 = h1_eqn(dd_bodyX,dd_bodyY,dd_bodyZ,d_bodyRoll, d_bodyPitch, d_bodyYaw,mx,my,mz);
        H = [H;H1];
        h = [h;h1];
        
        R = [R,r_accelerometer,r_accelerometer,r_accelerometer,r_gyroscope,r_gyroscope,r_gyroscope,r_magnetometer,r_magnetometer,r_magnetometer];
        
    end
    
    %%=================================================================
    %%barometer
    %%=================================================================
    
    if z02Avail(i) == 1
        z02 = [body_barometer(i)];
        zk = [zk; z03];
        
        H2 = H2_matrix(bodyZ);
        h2 = h2_eqn(bodyZ);
        H = [H;H2];
        h = [h;h2];
        
        R = [R,r_barotometer];
        
    end
    
    %%=================================================================
    %%gps
    %%=================================================================
    
    if z03Avail(i) == 1
        z03 = [body_gps_pos(i,:)';body_gps_vel(i,:)'];
        zk = [zk; z03];
        
        H3 = H3_matrix(bodyX, bodyY, d_bodyX, d_bodyY);
        h3 = h3_eqn();
        H = [H;H3];
        h = [h;h3];
        
        R = [R,r_gps_pos,r_gps_pos,r_gps_vel,r_gps_vel,];
    end
    
    %%=================================================================
    %%cameras fl fr bl br
    %%=================================================================
    
    if z04Avail(i) == 1
        z04 = [fl1(i,1);fl1(i,2)];
        zk = [zk; z04];
        
        H4 = H4_matrix();
        h4 = h4_eqn();
        H = [H;H4];
        h = [h;h4];
        
        R = [R,r_pixel,r_pixel];
    end
    
    if z05Avail(i) == 1
        z05 = [fl2(i,1);fl2(i,2)];
        zk = [zk; z05];
        
        H5 = H5_matrix();
        h5 = h5_eqn();
        H = [H;H5];
        h = [h;h5];
        
        R = [R,r_pixel,r_pixel];
    end
    
    if z06Avail(i) == 1
        z06 = [fl3(i,1);fl3(i,2)];
        zk = [zk; z06];
        
        H6 = H6_matrix();
        h6 = h6_eqn();
        H = [H;H6];
        h = [h;h6];
        
        R = [R,r_pixel,r_pixel];
    end
    
    if z07Avail(i) == 1
        z07 = [fl4(i,1);fl4(i,2)];
        zk = [zk; z07];
        
        H7 = H7_matrix();
        h7 = h7_eqn();
        H = [H;H7];
        h = [h;h7];
        
        R = [R,r_pixel,r_pixel];
    end
    
    if z08Avail(i) == 1
        z08 = [fr1(i,1);fr1(i,2)];
        zk = [zk; z08];
        
        H8 = H8_matrix();
        h8 = h8_eqn();
        H = [H;H8];
        h = [h;h8];
        
        R = [R,r_pixel,r_pixel];
    end
    
    if z09Avail(i) == 1
        z10 = [fr2(i,1);fr2(i,2)];
        zk = [zk; z10];
        
        H9 = H9_matrix();
        h9 = h9_eqn();
        H = [H;H9];
        h = [h;h9];
        
        R = [R,r_pixel,r_pixel];
    end
    
    if z10Avail(i) == 1
        z10 = [fr3(i,1);fr3(i,2)];
        zk = [zk; z10];
        
        H10 = H10_matrix();
        h10 = h10_eqn();
        H = [H;H10];
        h = [h;h10];
        
        R = [R,r_pixel,r_pixel];
    end
    
    if z11Avail(i) == 1
        z11 = [fr4(i,1);fr4(i,2)];
        zk = [zk; z11];
        
        H10 = H11_matrix();
        h11 = h11_eqn();
        H = [H;H11];
        h = [h;h11];
        
        R = [R,r_pixel,r_pixel];
    end
    
    if z12Avail(i) == 1
        z12 = [bl1(i,1),bl(i,2)];
        zk = [zk; z12];
        
        H10 = H12_matrix();
        h12 = h12_eqn();
        H = [H;H12];
        h = [h;h12];
        
        R = [R,r_pixel,r_pixel];
    end
    
    if z13Avail(i) == 1
        z13 = [bl2(i,1),bl2(i,2)];
        zk = [zk; z13];
        
        H10 = H13_matrix();
        h13 = h13_eqn();
        H = [H;H13];
        h = [h;h13];
        
        R = [R,r_pixel,r_pixel];
    end
    
    if z14Avail(i) == 1
        z14 = [bl3(i,1),bl3(i,2)];
        zk = [zk; z14];
        
        H10 = H14_matrix();
        h14 = h14_eqn();
        H = [H;H14];
        h = [h;h14];
        
        R = [R,r_pixel,r_pixel];
    end
    
    if z15Avail(i) == 1
        z15 = [bl4(i,1),bl4(i,2)];
        zk = [zk; z15];
        
        H10 = H15_matrix();
        h15 = h15_eqn();
        H = [H;H15];
        h = [h;h15];
        
        R = [R,r_pixel,r_pixel];
    end
    
    if z16Avail(i) == 1
        z16 = [br1(i,1),br1(i,2)];
        zk = [zk; z16];
        
        H10 = H16_matrix();
        h16 = h16_eqn();
        H = [H;H16];
        h = [h;h16];
        
        R = [R,r_pixel,r_pixel];
    end
    
    if z17Avail(i) == 1
        z17 = [br2(i,1),br2(i,2)];
        zk = [zk; z17];
        
        H10 = H17_matrix();
        h17 = h17_eqn();
        H = [H;H17];
        h = [h;h17];
        
        R = [R,r_pixel,r_pixel];
    end
    
    if z18Avail(i) == 1
        z18 = [br3(i,1),br3(i,2)];
        zk = [zk; z18];
        
        H10 = H18_matrix();
        h18 = h18_eqn();
        H = [H;H18];
        h = [h;h18];
        
        R = [R,r_pixel,r_pixel];
    end
    
    if z19Avail(i) == 1
        z19 = [br3(i,1),br3(i,2)];
        zk = [zk; z19];
        
        H10 = H19_matrix();
        h19 = h19_eqn();
        H = [H;H19];
        h = [h;h19];
        
        R = [R,r_pixel,r_pixel];
    end
    
    % Kalman Gain
    K = (P*H')/(H*P*H'+diag(R));
    % Correction
    states = states + K*(zk-h);
    % New covariance
    P = (I-K*H)*P;
    
end
