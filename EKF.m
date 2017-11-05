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
create_equations;       %create the measurement equations DONE
states = zeros(42,42);

mx = -0.1316;
my = 17.3801;
mz = -21.101;

%%=================================================================
%%FILTER
%%=================================================================

x_estimated_store = zeros(34,N);
x_actual_store = zeros(34,N);

Pcov_store = zeros(34,34,N);
p_store = zeros(34,34,N);
f_store = zeros(34,34,N);

N = 1800;
I = eye(42);


for i=1:1:N
    %%=================================================================
    %%Predict
    %%=================================================================
    
    zk = [];
    H = [];
    h = [];
    R = [];
    
    states = num2cell(states);
    [bodyX, bodyY, bodyZ, bodyRoll, bodyPitch, bodyYaw,LHipPitch, LHipYaw, LKneePitch, LAnklePitch,RHipPitch, RHipYaw, RKneePitch, RAnklePitch,d_bodyX, d_bodyY, d_bodyZ, d_bodyRoll, d_bodyPitch, d_bodyYaw,d_LHipPitch, d_LHipYaw, d_LKneePitch, d_LAnklePitch,d_RHipPitch, d_RHipYaw, d_RKneePitch, d_RAnklePitch,dd_bodyX, dd_bodyY, dd_bodyZ, dd_bodyRoll, dd_bodyPitch, dd_bodyYaw,dd_LHipPitch, dd_LHipYaw, dd_LKneePitch, dd_LAnklePitch,dd_RHipPitch, dd_RHipYaw, dd_RKneePitch, dd_RAnklePitch] = deal(states{:});
    states = state_predict_function(bodyX, bodyY, bodyZ, bodyRoll, bodyPitch, bodyYaw,LHipPitch, LHipYaw, LKneePitch, LAnklePitch,RHipPitch, RHipYaw, RKneePitch, RAnklePitch,d_bodyX, d_bodyY, d_bodyZ, d_bodyRoll, d_bodyPitch, d_bodyYaw,d_LHipPitch, d_LHipYaw, d_LKneePitch, d_LAnklePitch,d_RHipPitch, d_RHipYaw, d_RKneePitch, d_RAnklePitch,dd_bodyX, dd_bodyY, dd_bodyZ, dd_bodyRoll, dd_bodyPitch, dd_bodyYaw,dd_LHipPitch, dd_LHipYaw, dd_LKneePitch, dd_LAnklePitch,dd_RHipPitch, dd_RHipYaw, dd_RKneePitch, dd_RAnklePitch);
    x_estimated_store(:,i) = states;    
    
    %determining the F matrix
    Fmatrix  = Fmatrix_function(bodyX, bodyY, bodyZ, bodyRoll, bodyPitch, bodyYaw,LHipPitch, LHipYaw, LKneePitch, LAnklePitch,RHipPitch, RHipYaw, RKneePitch, RAnklePitch,d_bodyX, d_bodyY, d_bodyZ, d_bodyRoll, d_bodyPitch, d_bodyYaw,d_LHipPitch, d_LHipYaw, d_LKneePitch, d_LAnklePitch,d_RHipPitch, d_RHipYaw, d_RKneePitch, d_RAnklePitch,dd_bodyX, dd_bodyY, dd_bodyZ, dd_bodyRoll, dd_bodyPitch, dd_bodyYaw,dd_LHipPitch, dd_LHipYaw, dd_LKneePitch, dd_LAnklePitch,dd_RHipPitch, dd_RHipYaw, dd_RKneePitch, dd_RAnklePitch);
    f_store(:,:,i) = Fmatrix;
    
    %determining the P matrix
    P = (Fmatrix)*(P*(Fmatrix')) + Q;
    p_store(:,:,i) = P;
    
    
    %%=================================================================
    %%Update
    %%=================================================================
    
    if z01Avail(i) == 1
        z01 = [body_accel_x(i)';body_accel_y(i)';body_accel_z(i)';body_gyro_x(i)';body_gyro_y(i)';body_gyro_z(i)';body_mag_x(i)';body_mag_y(i)';body_mag_z(i)'];
        zk = [zk; z01];
        
        H1 = H1_matrix(dd_bodyX,dd_bodyY,dd_bodyZ,d_bodyRoll, d_bodyPitch, d_bodyYaw,mx,my,mz);
        h1 = h1_eqn(dd_bodyX,dd_bodyY,dd_bodyZ,d_bodyRoll, d_bodyPitch, d_bodyYaw,mx,my,mz);
        H = [H;H1];
        h = [h;h1];
        
        R = [R,r_accelerometer,r_accelerometer,r_accelerometer,r_gyroscope,r_gyroscope,r_gyroscope,r_magnetometer,r_magnetometer,r_magnetometer];
        
    end
    
    if z02Avail(i) == 1
        z02 = [];
        zk = [zk; z03];
        
        H2 = H2_matrix();
        h2 = h2_eqn();
        H = [H;H2];
        h = [h;h2];
        
    end
    
    if z03Avail(i) == 1
        z03 = [];
        zk = [zk; z03];
        
        H3 = H3_matrix();
        h3 = h3_eqn();
        H = [H;H3];
        h = [h;h3];
    end
    
    if z04Avail(i) == 1
        H = [H;H4];
        h = [h;h4];
    end
    
    if z05Avail(i) == 1
        H = [H;H5];
        h = [h;h5];
    end
    
    if z06Avail(i) == 1
        H = [H;H6];
        h = [h;h6];
    end
    
    if z07Avail(i) == 1
        H = [H;H7];
        h = [h;h7];
    end
    
    if z08Avail(i) == 1
        H = [H;H8];
        h = [h;h8];
    end
    
    if z09Avail(i) == 1
        H = [H;H9];
        h = [h;h9];
    end
    
    if z10Avail(i) == 1
        H = [H;H10];
        h = [h;h10];
    end
    
    if z11Avail(i) == 1
        H = [H;H11];
        h = [h;h11];
    end
    
    if z12Avail(i) == 1
        H = [H;H12];
        h = [h;h12];
    end
    
    if z13Avail(i) == 1
        H = [H;H13];
        h = [h;h13];
    end
    
    if z14Avail(i) == 1
        H = [H;H14];
        h = [h;h14];
    end
    
    if z15Avail(i) == 1
        H = [H;H15];
        h = [h;h15];
    end
    
    if z16Avail(i) == 1
        H = [H;H16];
        h = [h;h16];
    end
    
    if z17Avail(i) == 1
        H = [H;H17];
        h = [h;h17];
    end
    
    if z18Avail(i) == 1
        H = [H;H18];
        h = [h;h18];
    end
    
    if z19Avail(i) == 1
        H = [H;H19];
        h = [h;h19];
    end
    
    % Kalman Gain
    K = (P*H')/(H*P*H'+diag(R));
    % Correction
    states = states + K*(zk-h);
    % New covariance
    P = (I-K*H)*P;
    
end
