%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%symbols for the states same order as state vector
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%positional and anguar
syms bodyX bodyY bodyZ bodyRoll bodyPitch bodyYaw
syms LHipPitch LHipYaw LKneePitch LAnklePitch
syms RHipPitch RHipYaw RKneePitch RAnklePitch
%first derivative
syms d_bodyX d_bodyY d_bodyZ d_bodyRoll d_bodyPitch d_bodyYaw
syms d_LHipPitch d_LHipYaw d_LKneePitch d_LAnklePitch
syms d_RHipPitch d_RHipYaw d_RKneePitch d_RAnklePitch
%second derivative
syms dd_bodyX dd_bodyY dd_bodyZ dd_bodyRoll dd_bodyPitch dd_bodyYaw
syms dd_LHipPitch dd_LHipYaw dd_LKneePitch dd_LAnklePitch
syms dd_RHipPitch dd_RHipYaw dd_RKneePitch dd_RAnklePitch
%create symbolic state
X = [...
    bodyX; bodyY; bodyZ; bodyRoll; bodyPitch; bodyYaw;...
    LHipPitch; LHipYaw; LKneePitch; LAnklePitch;...
    RHipPitch; RHipYaw; RKneePitch; RAnklePitch;...
    d_bodyX; d_bodyY; d_bodyZ; d_bodyRoll; d_bodyPitch; d_bodyYaw;...
    d_LHipPitch; d_LHipYaw; d_LKneePitch; d_LAnklePitch;...
    d_RHipPitch; d_RHipYaw; d_RKneePitch; d_RAnklePitch;...
    dd_bodyX; dd_bodyY; dd_bodyZ; dd_bodyRoll; dd_bodyPitch; dd_bodyYaw;...
    dd_LHipPitch; dd_LHipYaw; dd_LKneePitch; dd_LAnklePitch;...
    dd_RHipPitch; dd_RHipYaw; dd_RKneePitch; dd_RAnklePitch...
];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Prediction of the states assuming acceleration is constant
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%body linear 
vec_pos = [bodyX; bodyY; bodyZ];
vec_vel = [d_bodyX; d_bodyY; d_bodyZ];
vec_acc = [dd_bodyX; dd_bodyY; dd_bodyZ];
%body angular     
vec_iomega = [bodyRoll; bodyPitch; bodyYaw];
vec_omega = [d_bodyRoll; d_bodyPitch; d_bodyYaw];
vec_domega = [dd_bodyRoll; dd_bodyPitch; dd_bodyYaw];
%legs angles
vec_legs = [LHipPitch; LHipYaw; LKneePitch; LAnklePitch;...
 RHipPitch; RHipYaw; RKneePitch; RAnklePitch];
%legs dangles
vec_dlegs = [d_LHipPitch; d_LHipYaw; d_LKneePitch; d_LAnklePitch;...
 d_RHipPitch; d_RHipYaw; d_RKneePitch; d_RAnklePitch];
%legs ddangles
vec_ddlegs = [dd_LHipPitch; dd_LHipYaw; dd_LKneePitch; dd_LAnklePitch;...
    dd_RHipPitch; dd_RHipYaw; dd_RKneePitch; dd_RAnklePitch];



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%prediction equation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%redefining the sampling interval
ts = 0.01;
%preditc body linear
predict_eq_linAcc = vec_acc;                    % assume constant lin accel
predict_eq_linVel = vec_vel + st*vec_acc;       % linear velocity
predict_eq_linPos = vec_pos + st*vec_pos;       % linear position
%predict body angular
predict_eq_angAcc = vec_domega;                 % assume constant ang accel
predict_eq_angVel = vec_omega + st*vec_domega;  % angular velocity
predict_eq_angPos = vec_iomega + st*vec_omega;  % angular position
%predict legs
predict_eq_legsAcc = vec_ddlegs;                %leg angles constant accel
predict_eq_legsVel = vec_dlegs + st*vec_ddlegs; %leg velocity
predict_eq_legsPos = vec_legs + st*vec_dlegs;   %leg position angles
%make prediction vector
predict_eq = [ ...
    predict_eq_linPos;...
    predict_eq_angPos;...
    predict_eq_legsPos;...
    predict_eq_linVel;...
    predict_eq_angVel;...
    predict_eq_legsVel;...
    predict_eq_linAcc;...
    predict_eq_angAcc;...
    predict_eq_legsAcc...
];
%Create state_prediction_function
predict_eq = simplify(predict_eq);
matlabFunction(predict_eq,'file','state_prediction_function');
%Create F_matrix_function
Fmatrix_eq=jacobian(predict_eq,X);
matlabFunction(Fmatrix_eq,'file','F_matrix_function');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%creating syms for measurements
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%magnotometer
syms mx my mz
vec_mag = [mx; my; mz];
%barometer
syms bar
%gps
syms gpsx gpsy
syms gpsvel gpshead

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%h1 IMU 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
RB_I_linear = [];
RB_I_angular = [];

[body_accel_x;body_accel_y;body_accel_z] = vec_acc;             
[body_gyro_x;body_gyro_y;body_gyro_z] = vec_omega;

% h1 Equation
h1_eqn = [AccSensor; GyroSensor];
h1_eqn = simplify(h1_eqn);
matlabFunction(h1_eqn,'file','h1_eqn');
% h1 Jacobian
H1_matrix_eq=jacobian(h1_eqn,X);
matlabFunction(H1_matrix_eq,'file','H1_matrix');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%h2 barometer DONE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
body_barometer = 1013.25*(1-bodyZ/44307.7)^5.25;
% h2 Equation
h2_eqn = [PressSens];
matlabFunction(h2_eqn,'file','h2_eqn');
% h2 Jacobian
H2_matrix_eq=jacobian(h2_eqn,X);
matlabFunction(H2_matrix_eq,'file','H2_matrix');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%h3 GPS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GPSPos = [-x;y];
% GPSVel = [-dx;dy];
% GPSCourse = [angs(3)];
% % h3 Equation
% h3_eqn = [GPSPos;GPSVel;GPSCourse];
% matlabFunction(h3_eqn,'file','h3_eqn');
% % h3 Jacobian
% H3_matrix_eq=jacobian(h3_eqn,X);
% matlabFunction(H3_matrix_eq,'file','H3_matrix');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%import camera goodies
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%loading the camera models as stereo calibration sessions...
FrontStereoSession = load('FRONTcalibrationSession.mat');
BackStereoSession = load('BACKcalibrationSession.mat');
%%defining the camera matrices.
%front right
FRCameraMatrix = cameraMatrix(FrontStereoSession.calibrationSession.CameraParameters.CameraParameters2,...
FrontStereoSession.calibrationSession.CameraParameters.RotationOfCamera2,...
FrontStereoSession.calibrationSession.CameraParameters.TranslationOfCamera2);
FRCameraMatrix = FRCameraMatrix';
%front left
FLCameraMatrix = cameraMatrix(FrontStereoSession.calibrationSession.CameraParameters.CameraParameters1,eye(3),zeros(3,1));
FLCameraMatrix = FLCameraMatrix';
%back right
BRCameraMatrix = cameraMatrix(BackStereoSession.calibrationSession.CameraParameters.CameraParameters1,eye(3),zeros(3,1));
BRCameraMatrix = BRCameraMatrix';
%back left
BLCameraMatrix = cameraMatrix(BackStereoSession.calibrationSession.CameraParameters.CameraParameters2,...
BackStereoSession.calibrationSession.CameraParameters.RotationOfCamera2,...
BackStereoSession.calibrationSession.CameraParameters.TranslationOfCamera2);
BLCameraMatrix = BLCameraMatrix';
%work in meters

%model
vec_thigh = [0;0;400];
vec_calf = [0;0;400];
vec_foot = [200;0;0];
vec_body_right = [-10;-100;100];
vec_body_left = [-10;100;100];

% Front Points WRT front left camera frame
%       point 1 right knee
%       point 2 left knee
%       point 3 right foot
%       point 4 left foot
% Back Points WRT back left camera frame
%       point 1 right calf
%       point 2 left calf
%       point 3 right heel
%       point 4 left heel


%%still needs transformation from body frame to camera frame

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       FL POINT 1       %%%%%%         H4
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
FL_point1_xyz = vec_body_right + eulrot(0,RHipPitch,RHipYaw)*vec_thigh + [40;0;0];
FL_point1_xyz = [FL_point1_xyz(2);FL_point1_xyz(1);FL_point1_xyz(3);1];
FL_point1_pixels = FLCameraMatrix(1:2,:) * FL_point1_xyz./repmat(FLCameraMatrix(3,:) * FL_point1_xyz,2,1);
% h4 Equation
h4_eqn = FL_point1_pixels;
matlabFunction(h4_eqn,'file','h4_eqn');
% h4 Jacobian
H4_matrix_eq=jacobian(h4_eqn,X);
matlabFunction(H4_matrix_eq,'file','H4_matrix');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       FL POINT 2       %%%%%%         H5
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
FL_point2_xyz = vec_body_left + eulrot(0,LHipPitch,LHipYaw)*vec_thigh + [40;0;0]; 
FL_point2_pixels = FLCameraMatrix * FL_point2_xyz; 
% h5 Equation
h5_eqn = [FL_point1_pixels];
matlabFunction(h5_eqn,'file','h5_eqn');
% h5 Jacobian
H4_matrix_eq=jacobian(h5_eqn,X);
matlabFunction(H4_matrix_eq,'file','H5_matrix');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       FL POINT 3       %%%%%%         H6
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
FL_point3_xyz = FL_point1_xyz + eulrot(0,RKneePitch,0)*vec_calf + eulrot(0,RAnklePitch,0)*vec_calf + [40;0;0]; 
FL_point3_pixels = FLCameraMatrix * FL_point3_xyz;
% h6 Equation
h6_eqn = [FL_point3_pixels];
matlabFunction(h6_eqn,'file','h6_eqn');
% h6 Jacobian
H6_matrix_eq=jacobian(h6_eqn,X);
matlabFunction(H4_matrix_eq,'file','H6_matrix');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       FL POINT 4       %%%%%%         H7
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
FL_point4_xyz = FL_point2_xyz + eulrot(0,LKneePitch,0)*vec_calf + eulrot(0,LAnklePitch,0)*vec_foot + [40;0;0]; 
FL_point4_pixels = FLCameraMatrix * FL_point4_xyz;
% h4 Equation
h7_eqn = [FL_point4_pixels];
matlabFunction(h7_eqn,'file','h7_eqn');
% h4 Jacobian
H7_matrix_eq=jacobian(h7_eqn,X);
matlabFunction(H7_matrix_eq,'file','H7_matrix');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       FR POINT 1       %%%%%%         H8
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
FR_point1_xyz = vec_body_right + eulrot(0,RHipPitch,RHipYaw)*vec_thigh;
FR_point1_pixels = FRCameraMatrix * FR_point1_xyz;
% h8 Equation
h8_eqn = [FR_point1_pixels];
matlabFunction(h8_eqn,'file','h8_eqn');
% h8 Jacobian
H8_matrix_eq=jacobian(h8_eqn,X);
matlabFunction(H8_matrix_eq,'file','H8_matrix');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       FR POINT 2       %%%%%%         H9
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
FR_point2_xyz = vec_body_left + eulrot(0,LHipPitch,LHipYaw)*vec_thigh;
FR_point2_pixels = FRCameraMatrix * FR_point2_xyz;
% h9 Equation
h9_eqn = [FR_point2_pixels];
matlabFunction(h9_eqn,'file','h9_eqn');
% h9 Jacobian
H9_matrix_eq=jacobian(h9_eqn,X);
matlabFunction(H9_matrix_eq,'file','H9_matrix');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       FR POINT 3       %%%%%%         H10
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
FR_point3_xyz = FR_point1_xyz + eulrot(0,RKneePitch,0)*vec_calf + eulrot(0,RAnklePitch,0)*vec_calf;
FR_point3_pixels = FRCameraMatrix * FR_point3_xyz;
% h10 Equation
h10_eqn = [FR_point3_pixels];
matlabFunction(h10_eqn,'file','h10_eqn');
% h10 Jacobian
H10_matrix_eq=jacobian(h10_eqn,X);
matlabFunction(H10_matrix_eq,'file','H10_matrix');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       FR POINT 4       %%%%%%         H11
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
FR_point4_xyz = FR_point2_xyz + eulrot(0,LKneePitch,0)*vec_calf + eulrot(0,LAnklePitch,0)*vec_foot;
FR_point4_pixels = FRCameraMatrix * FR_point4_xyz;
% h4 Equation
h11_eqn = [FR_point4_pixels];
matlabFunction(h11_eqn,'file','h11_eqn');
% h11 Jacobian
H11_matrix_eq=jacobian(h11_eqn,X);
matlabFunction(H11_matrix_eq,'file','H11_matrix');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       BL POINT 1       %%%%%%         H12
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
BL_point1_xyz = vec_body_right + eulrot(0,RHipPitch,RHipYaw)*vec_thigh + ...
0.5*eulrot(0,RKneePitch,0)*vec_calf + [40;0;0]; 
BL_point1_pixels = BLCameraMatrix * BL_point1_xyz;
% h12 Equation
h12_eqn = [BL_point1_pixels];
matlabFunction(h12_eqn,'file','h12_eqn');
% h12 Jacobian
H12_matrix_eq=jacobian(h12_eqn,X);
matlabFunction(H12_matrix_eq,'file','H12_matrix');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       BL POINT 2       %%%%%%         H13
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
BL_point2_xyz = vec_body_left + eulrot(0,RHipPitch,RHipYaw)*vec_thigh + ...
0.5*eulrot(0,RKneePitch,0)*vec_calf + [40;0;0]; 
BL_point2_pixels = BLCameraMatrix * BL_point2_xyz;
% h13 Equation
h13_eqn = [BL_point2_pixels];
matlabFunction(h13_eqn,'file','h13_eqn');
% h13 Jacobian
H13_matrix_eq=jacobian(h13_eqn,X);
matlabFunction(H13_matrix_eq,'file','H13_matrix');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       BL POINT 3       %%%%%%         H14
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
BL_point3_xyz = vec_body_right + eulrot(0,RHipPitch,RHipYaw)*vec_thigh + ...
eulrot(0,RKneePitch,0)*(vec_calf + [0;0;15]) + [40;0;0]; 
BL_point3_pixels = BLCameraMatrix * BL_point3_xyz;
% h14 Equation
h14_eqn = [BL_point3_pixels];
matlabFunction(h14_eqn,'file','h14_eqn');
% h14 Jacobian
H14_matrix_eq=jacobian(h14_eqn,X);
matlabFunction(H14_matrix_eq,'file','H14_matrix');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       BL POINT 4       %%%%%%         H15
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
BL_point4_xyz = vec_body_left + eulrot(0,RHipPitch,RHipYaw)*vec_thigh + ...
eulrot(0,RKneePitch,0)*(vec_calf + [0;0;15]) + [40;0;0]; 
BL_point4_pixels = BLCameraMatrix * BL_point4_xyz;
% h15 Equation
h15_eqn = [BL_point4_pixels];
matlabFunction(h15_eqn,'file','h15_eqn');
% h15 Jacobian
H15_matrix_eq=jacobian(h15_eqn,X);
matlabFunction(H15_matrix_eq,'file','H15_matrix');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       BR POINT 1       %%%%%%         H16
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
BR_point1_xyz = vec_body_right + eulrot(0,RHipPitch,RHipYaw)*vec_thigh + ...
0.5*eulrot(0,RKneePitch,0)*vec_calf; 
BR_point1_pixels = BRCameraMatrix * BR_point1_xyz;
% h16 Equation
h16_eqn = [BR_point1_pixels];
matlabFunction(h16_eqn,'file','h16_eqn');
% h16 Jacobian
H16_matrix_eq=jacobian(h16_eqn,X);
matlabFunction(H16_matrix_eq,'file','H16_matrix');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       BR POINT 2       %%%%%%         H17
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
BR_point2_xyz = vec_body_left + eulrot(0,RHipPitch,RHipYaw)*vec_thigh + ...
0.5*eulrot(0,RKneePitch,0)*vec_calf;
BR_point2_pixels = BRCameraMatrix * BR_point2_xyz;
% h17 Equation
h17_eqn = [BR_point2_pixels];
matlabFunction(h17_eqn,'file','h17_eqn');
% h17 Jacobian
H17_matrix_eq=jacobian(h17_eqn,X);
matlabFunction(H17_matrix_eq,'file','H17_matrix');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       BR POINT 3       %%%%%%         H18
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
BR_point3_xyz = vec_body_right + eulrot(0,RHipPitch,RHipYaw)*vec_thigh + ...
eulrot(0,RKneePitch,0)*(vec_calf + [0;0;15]);
BR_point3_pixels = BRCameraMatrix * BR_point3_xyz;
% h18 Equation
h18_eqn = [BR_point3_pixels];
matlabFunction(h18_eqn,'file','h18_eqn');
% h18 Jacobian
H18_matrix_eq=jacobian(h18_eqn,X);
matlabFunction(H18_matrix_eq,'file','H18_matrix');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       BR POINT 4       %%%%%%         H19
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
BR_point4_xyz = vec_body_left + eulrot(0,RHipPitch,RHipYaw)*vec_thigh + ...
eulrot(0,RKneePitch,0)*(vec_calf + [0;0;15]);
BR_point4_pixels = BRCameraMatrix * BR_point4_xyz;
% h19 Equation
h19_eqn = [BR_point4_pixels];
matlabFunction(h19_eqn,'file','h19_eqn');
% h19 Jacobian
H19_matrix_eq=jacobian(h19_eqn,X);
matlabFunction(H19_matrix_eq,'file','H19_matrix');

