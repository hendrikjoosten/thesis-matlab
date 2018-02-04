%%=================================================================
%%admin
%%=================================================================

clear all
close all
clc

%%=================================================================
%%preprocess data, derive eqautions, make PQR
%%=================================================================

create_PQR;             %create important matrices and initialise R values
create_data;            %create the measurement vectors DONE    
create_equations;       %create the measurement equations DONE

states = zeros(42,1);



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

storez = [];


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
    states = state_prediction_function(bodyX, bodyY, bodyZ, bodyRoll, bodyPitch, bodyYaw, LHipPitch, LHipYaw, LKneePitch, LAnklePitch,RHipPitch, RHipYaw, RKneePitch, RAnklePitch,d_bodyX, d_bodyY, d_bodyZ, d_bodyRoll, d_bodyPitch, d_bodyYaw,d_LHipPitch, d_LHipYaw, d_LKneePitch, d_LAnklePitch,d_RHipPitch, d_RHipYaw, d_RKneePitch, d_RAnklePitch,dd_bodyX, dd_bodyY, dd_bodyZ, dd_bodyRoll, dd_bodyPitch, dd_bodyYaw,dd_LHipPitch, dd_LHipYaw, dd_LKneePitch, dd_LAnklePitch,dd_RHipPitch, dd_RHipYaw, dd_RKneePitch, dd_RAnklePitch);
    x_estimated_store(:,i) = states;    
    
    %determining the F matrix
    Fmatrix  = F_matrix_function();
    f_store(:,:,i) = Fmatrix;
    
    %determining the P matrix
    P = (Fmatrix)*(P*(Fmatrix')) + Q*0.000001;
    p_store(:,:,i) = P;
    
    
    %%=================================================================
    %%Update
    %%=================================================================
    
    %%=================================================================
    %%acceleromteter gyroscope, magnetometer
    %%=================================================================
    
    if z01Avail(i) == 1
        z01 = [body_accel_x(i);body_accel_y(i);body_accel_z(i);body_gyro_x(i);body_gyro_y(i);body_gyro_z(i)];
        zk = [zk; z01];
        
        H1 = H1_matrix();
        h1 = h1_eqn(d_bodyYaw,d_bodyRoll,d_bodyPitch,dd_bodyX,dd_bodyY,dd_bodyZ);
        H = [H;H1];
        h = [h;h1];
        
        R = [R,r_accelerometer,r_accelerometer,r_accelerometer,r_gyroscope,r_gyroscope,r_gyroscope];
        
    end
    
    %%=================================================================
    %%barometer
    %%=================================================================
    
    if z02Avail(i) == 1
        z02 = body_barometer(i);
        zk = [zk; z02];
        
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
        
        H3 = H3_matrix();
        h3 = h3_eqn(bodyX, bodyY, d_bodyX, d_bodyY);
        H = [H;H3];
        h = [h;h3];
        
        R = [R,r_gps_pos,r_gps_pos,r_gps_vel,r_gps_vel];
    end
    
    %%=================================================================
    %%cameras fl fr bl br
    %%=================================================================
    
    if z04Avail(i) == 1
        z04 = [fl1(i,1);fl1(i,2)];
        zk = [zk; z04];
        
        H4 = H4_matrix(RHipPitch);
        h4 = h4_eqn(RHipPitch);
        H = [H;H4];
        h = [h;h4];
        
        R = [R,r_pixel,r_pixel];
    end
    
    if z05Avail(i) == 1
        z05 = [fl2(i,1);fl2(i,2)];
        zk = [zk; z05];
        
        H5 = H5_matrix(LHipPitch);
        h5 = h5_eqn(LHipPitch);
        H = [H;H5];
        h = [h;h5];
        
        R = [R,r_pixel,r_pixel];
    end
    
    if z06Avail(i) == 1
        z06 = [fl3(i,1);fl3(i,2)];
        zk = [zk; z06];
        
        H6 = H6_matrix(RAnklePitch,RHipPitch,RKneePitch);
        h6 = h6_eqn(RAnklePitch,RHipPitch,RKneePitch);
        H = [H;H6];
        h = [h;h6];
        
        R = [R,r_pixel,r_pixel];
    end
    
    if z07Avail(i) == 1
        z07 = [fl4(i,1);fl4(i,2)];
        zk = [zk; z07];
        
        H7 = H7_matrix(LAnklePitch,LHipPitch,LKneePitch);
        h7 = h7_eqn(LAnklePitch,LHipPitch,LKneePitch);
        H = [H;H7];
        h = [h;h7];
        
        R = [R,r_pixel,r_pixel];
    end
    
    if z08Avail(i) == 1
        z08 = [fr1(i,1);fr1(i,2)];
        zk = [zk; z08];
        
        H8 = H8_matrix(RHipPitch);
        h8 = h8_eqn(RHipPitch);
        H = [H;H8];
        h = [h;h8];
        
        R = [R,r_pixel,r_pixel];
    end
    
    if z09Avail(i) == 1
        z09 = [fr2(i,1);fr2(i,2)];
        zk = [zk; z09];
        
        H9 = H9_matrix(LHipPitch);
        h9 = h9_eqn(LHipPitch);
        H = [H;H9];
        h = [h;h9];
        
        R = [R,r_pixel,r_pixel];
    end
    
    if z10Avail(i) == 1
        z10 = [fr3(i,1);fr3(i,2)];
        zk = [zk; z10];
        
        H10 = H10_matrix(RAnklePitch,RHipPitch,RKneePitch);
        h10 = h10_eqn(RAnklePitch,RHipPitch,RKneePitch);
        H = [H;H10];
        h = [h;h10];
        
        R = [R,r_pixel,r_pixel];
    end
    
    if z11Avail(i) == 1
        z11 = [fr4(i,1);fr4(i,2)];
        zk = [zk; z11];
        
        H11 = H11_matrix(LAnklePitch,LHipPitch,LKneePitch);
        h11 = h11_eqn(LAnklePitch,LHipPitch,LKneePitch);
        H = [H;H11];
        h = [h;h11];
        
        R = [R,r_pixel,r_pixel];
    end
    
    if z12Avail(i) == 1
        z12 = [bl1(i,1);bl1(i,2)];
        zk = [zk; z12];
        
        H12 = H12_matrix(RHipPitch,RKneePitch);
        h12 = h12_eqn(RHipPitch,RKneePitch);
        H = [H;H12];
        h = [h;h12];
        
        R = [R,r_pixel,r_pixel];
    end
    
    if z13Avail(i) == 1         %left calf
        z13 = [bl2(i,1);bl2(i,2)];
        zk = [zk; z13];
        
        H13 = H13_matrix(LHipPitch,LKneePitch);
        h13 = h13_eqn(LHipPitch,LKneePitch);
        H = [H;H13];
        h = [h;h13];
        
        R = [R,r_pixel,r_pixel];
    end
    
    if z14Avail(i) == 1
        z14 = [bl3(i,1);bl3(i,2)];
        zk = [zk; z14];
        
        H14 = H14_matrix(RHipPitch,RKneePitch);
        h14 = h14_eqn(RHipPitch,RKneePitch);
        H = [H;H14];
        h = [h;h14];
        
        R = [R,r_pixel,r_pixel];
    end
    
    if z15Avail(i) == 1
        z15 = [bl4(i,1);bl4(i,2)];
        zk = [zk; z15];
        
        H15 = H15_matrix(LHipPitch,LKneePitch);
        h15 = h15_eqn(LHipPitch,LKneePitch);
        H = [H;H15];
        h = [h;h15];
        
        R = [R,r_pixel,r_pixel];
    end
    
    if z16Avail(i) == 1
        z16 = [br1(i,1);br1(i,2)];
        zk = [zk; z16];
        
        H16 = H16_matrix(RHipPitch,RKneePitch);
        h16 = h16_eqn(RHipPitch,RKneePitch);
        H = [H;H16];
        h = [h;h16];
        
        R = [R,r_pixel,r_pixel];
    end
    
    if z17Avail(i) == 1
        z17 = [br2(i,1);br2(i,2)];
        zk = [zk; z17];
        
        H17 = H17_matrix(LHipPitch,LKneePitch);
        h17 = h17_eqn(LHipPitch,LKneePitch);
        H = [H;H17];
        h = [h;h17];
        
        R = [R,r_pixel,r_pixel];
    end
    
    if z18Avail(i) == 1
        z18 = [br3(i,1);br3(i,2)];
        zk = [zk; z18];
        
        H18 = H18_matrix(RHipPitch,RKneePitch);
        h18 = h18_eqn(RHipPitch,RKneePitch);
        H = [H;H18];
        h = [h;h18];
        
        R = [R,r_pixel,r_pixel];
    end
    
    if z19Avail(i) == 1
        z19 = [br3(i,1);br3(i,2)];
        zk = [zk; z19];
        
        H19 = H19_matrix(LHipPitch,LKneePitch);
        h19 = h19_eqn(LHipPitch,LKneePitch);
        H = [H;H19];
        h = [h;h19];
        
        R = [R,r_pixel,r_pixel];
    end
    
    % Kalman Gain
    K = (P*H')/(H*P*H' + diag(R));
   
    % Correction
    states = states + K *(zk-h);
    x_actual_store(:,i) = states;
    
    % New covariance
    P = (I-K*H)*P;
    
end

figure(1)
subplot(4,4,1)
plot(abs(x_estimated_store(1,:)));
title('Estimated value of state 1')
subplot(4,4,2)
plot(abs(x_estimated_store(2,:)));
title('Estimated value of state 2')
subplot(4,4,3)
plot(abs(x_estimated_store(3,:)));
title('Estimated value of state 3')
subplot(4,4,4)
plot(abs(x_estimated_store(4,:)));
title('Estimated value of state 4')
subplot(4,4,5)
plot(abs(x_estimated_store(5,:)));
title('Estimated value of state 5')
subplot(4,4,6)
plot(abs(x_estimated_store(6,:)));
title('Estimated value of state 6')
subplot(4,4,7)
plot(abs(x_estimated_store(7,:)));
title('Estimated value of state 7')
subplot(4,4,8)
plot(abs(x_estimated_store(8,:)));
title('Estimated value of state 8')
subplot(4,4,9)
plot(abs(x_estimated_store(9,:)));
title('Estimated value of state 9')
subplot(4,4,10)
plot(abs(x_estimated_store(10,:)));
title('Estimated value of state 10')
subplot(4,4,11)
plot(abs(x_estimated_store(11,:)));
title('Estimated value of state 11')
subplot(4,4,12)
plot(abs(x_estimated_store(12,:)));
title('Estimated value of state 12')
subplot(4,4,13)
plot(abs(x_estimated_store(13,:)));
title('Estimated value of state 13')
subplot(4,4,14)
plot(abs(x_estimated_store(14,:)));
title('Estimated value of state 14')


figure(2)
subplot(4,4,1)
plot(abs(x_actual_store(1,:)));
title('Actual value of state 1')
subplot(4,4,2)
plot(abs(x_actual_store(2,:)));
title('Actual value of state 2')
subplot(4,4,3)
plot(abs(x_actual_store(3,:)));
title('Actual value of state 3')
subplot(4,4,4)
plot(abs(x_actual_store(4,:)));
title('Actual value of state 4')
subplot(4,4,5)
plot(abs(x_actual_store(5,:)));
title('Actual value of state 5')
subplot(4,4,6)
plot(abs(x_actual_store(6,:)));
title('Actual value of state 6')
subplot(4,4,7)
plot(abs(x_actual_store(7,:)));
title('Actual value of state 7')
subplot(4,4,8)
plot(abs(x_actual_store(8,:)));
title('Actual value of state 8')
subplot(4,4,9)
plot(abs(x_actual_store(9,:)));
title('Actual value of state 9')
subplot(4,4,10)
plot(abs(x_actual_store(10,:)));
title('Actual value of state 10')
subplot(4,4,11)
plot(abs(x_actual_store(11,:)));
title('Actual value of state 11')
subplot(4,4,12)
plot(abs(x_actual_store(12,:)));
title('Actual value of state 12')
subplot(4,4,13)
plot(abs(x_actual_store(13,:)));
title('Actual value of state 13')
subplot(4,4,14)
plot(abs(x_actual_store(14,:)));
title('Actual value of state 14')


