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
st = 0.01;
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
%h1 IMU 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

temp_acc = vec_acc;             
temp_gyro = vec_omega;
%h1 Equation
h1_eqn = [temp_acc; temp_gyro];
matlabFunction(h1_eqn,'file','h1_eqn');
%h1 Jacobian
H1_matrix_eq=jacobian(h1_eqn,X);
matlabFunction(H1_matrix_eq,'file','H1_matrix');
clear h1_eqn H1_matrix_eq

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%h2 barometer DONE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

temp_barometer = 1013.25*(1-bodyZ/44307.7)^5.25;
% h2 Equation
h2_eqn = temp_barometer;
matlabFunction(h2_eqn,'file','h2_eqn');
% h2 Jacobian
H2_matrix_eq=jacobian(h2_eqn,X);
matlabFunction(H2_matrix_eq,'file','H2_matrix');
clear h2_eqn H2_matrix_eq

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%h3 GPS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

temp_latlong = [-bodyX;bodyY];
temp_vel = [-d_bodyX;d_bodyY];

% h3 Equation
h3_eqn = [temp_latlong;temp_vel];
matlabFunction(h3_eqn,'file','h3_eqn');
% h3 Jacobian
H3_matrix_eq=jacobian(h3_eqn,X);
matlabFunction(H3_matrix_eq,'file','H3_matrix');
clear h3_eqn H3_matrix_eq

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
vec_thigh = [0;0;400]/1000;
vec_calf = [0;0;400]/1000;
vec_foot = [200;0;0]/1000;
vec_body_right = [-10;-100;100]/1000;
vec_body_left = [-10;100;100]/1000;

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
%%%%%       front
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       FL POINT 1       %%%%%%         H4 RIGHT KNEE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
FL_point1_xyz = vec_body_right + eulrot(0,RHipPitch,RHipYaw)*vec_thigh;

FL_point1_xyz = [FL_point1_xyz(2);FL_point1_xyz(1);FL_point1_xyz(3);1];

FL_point1_pixels = FLCameraMatrix(1:2,:) * FL_point1_xyz./repmat(FLCameraMatrix(3,:) * FL_point1_xyz,2,1);

% h4 Equation
h4_eqn = FL_point1_pixels;
matlabFunction(h4_eqn,'file','h4_eqn');
% h4 Jacobian
H4_matrix_eq=jacobian(h4_eqn,X);
matlabFunction(H4_matrix_eq,'file','H4_matrix');
clear h4_eqn H4_matrix_eq

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       FL POINT 2       %%%%%%         H5      LEFT KNEE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
FL_point2_xyz = vec_body_left + eulrot(0,LHipPitch,LHipYaw)*vec_thigh;

FL_point2_xyz = [FL_point2_xyz(2);FL_point2_xyz(1);FL_point2_xyz(3);1];

FL_point2_pixels = FLCameraMatrix(1:2,:) * FL_point2_xyz./repmat(FLCameraMatrix(3,:) * FL_point2_xyz,2,1);

% h5 Equation
h5_eqn = FL_point2_pixels;
matlabFunction(h5_eqn,'file','h5_eqn');
% h5 Jacobian
H5_matrix_eq=jacobian(h5_eqn,X);
matlabFunction(H5_matrix_eq,'file','H5_matrix');
clear h5_eqn H5_matrix_eq

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       FL POINT 3       %%%%%%         H6      RIGHT TOE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
FL_point3_xyz = vec_body_right + eulrot(0,RHipPitch,RHipYaw)*vec_thigh ...
+ eulrot(0,RKneePitch,0)*vec_calf + eulrot(0,RAnklePitch,0)*vec_foot; 

FL_point3_xyz = [FL_point3_xyz(2);FL_point3_xyz(1);FL_point3_xyz(3);1];

FL_point3_pixels = FLCameraMatrix(1:2,:) * FL_point3_xyz./repmat(FLCameraMatrix(3,:) * FL_point3_xyz,2,1);

% h6 Equation
h6_eqn = FL_point3_pixels;
matlabFunction(h6_eqn,'file','h6_eqn');
% h6 Jacobian
H6_matrix_eq=jacobian(h6_eqn,X);
matlabFunction(H6_matrix_eq,'file','H6_matrix');
clear h6_eqn H6_matrix_eq

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       FL POINT 4       %%%%%%         H7          LEFT TOE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
FL_point4_xyz = vec_body_left + eulrot(0,LHipPitch,LHipYaw)*vec_thigh ...
+ eulrot(0,LKneePitch,0)*vec_calf + eulrot(0,LAnklePitch,0)*vec_foot; 

FL_point4_xyz = [FL_point4_xyz(2);FL_point4_xyz(1);FL_point4_xyz(3);1];

FL_point4_pixels = FLCameraMatrix(1:2,:) * FL_point4_xyz./repmat(FLCameraMatrix(3,:) * FL_point4_xyz,2,1);

% h7 Equation
h7_eqn = FL_point4_pixels;
matlabFunction(h7_eqn,'file','h7_eqn');
% h7 Jacobian
H7_matrix_eq=jacobian(h7_eqn,X);
matlabFunction(H7_matrix_eq,'file','H7_matrix');
clear h7_eqn H7_matrix_eq


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       FR POINT 1       %%%%%%         H8          RIGHT KNEE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
FR_point1_xyz = vec_body_right + eulrot(0,RHipPitch,RHipYaw)*vec_thigh;

FR_point1_xyz = [FR_point1_xyz(2);FR_point1_xyz(1);FR_point1_xyz(3);1];

FR_point1_pixels = FRCameraMatrix(1:2,:) * FR_point1_xyz./repmat(FRCameraMatrix(3,:) * FR_point1_xyz,2,1);


% h8 Equation
h8_eqn = FR_point1_pixels;
matlabFunction(h8_eqn,'file','h8_eqn');
% h8 Jacobian
H8_matrix_eq=jacobian(h8_eqn,X);
matlabFunction(H8_matrix_eq,'file','H8_matrix');
clear h8_eqn H8_matrix_eq

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       FR POINT 2       %%%%%%         H9          LEFT KNEE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
FR_point2_xyz = vec_body_left + eulrot(0,LHipPitch,LHipYaw)*vec_thigh;

FR_point2_xyz = [FR_point2_xyz(2);FR_point2_xyz(1);FR_point2_xyz(3);1];

FR_point2_pixels = FRCameraMatrix(1:2,:) * FR_point2_xyz./repmat(FRCameraMatrix(3,:) * FR_point2_xyz,2,1);


% h9 Equation
h9_eqn = FR_point2_pixels;
matlabFunction(h9_eqn,'file','h9_eqn');
% h9 Jacobian
H9_matrix_eq=jacobian(h9_eqn,X);
matlabFunction(H9_matrix_eq,'file','H9_matrix');
clear h9_eqn H9_matrix_eq

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       FR POINT 3       %%%%%%         H10         RIGHT TOE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
FR_point3_xyz = vec_body_right + eulrot(0,RHipPitch,RHipYaw)*vec_thigh ...
    + eulrot(0,RKneePitch,0)*vec_calf + eulrot(0,RAnklePitch,0)*vec_foot;

FR_point3_xyz = [FR_point3_xyz(2);FR_point3_xyz(1);FR_point3_xyz(3);1];

FR_point3_pixels = FRCameraMatrix(1:2,:) * FR_point3_xyz./repmat(FRCameraMatrix(3,:) * FR_point3_xyz,2,1);


% h10 Equation
h10_eqn = FR_point3_pixels;
matlabFunction(h10_eqn,'file','h10_eqn');
% h10 Jacobian
H10_matrix_eq=jacobian(h10_eqn,X);
matlabFunction(H10_matrix_eq,'file','H10_matrix');
clear h10_eqn H10_matrix_eq

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       FR POINT 4       %%%%%%         H11         LEFT TOE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
FR_point4_xyz = vec_body_left + eulrot(0,LHipPitch,LHipYaw)*vec_thigh ...
    + eulrot(0,LKneePitch,0)*vec_calf + eulrot(0,LAnklePitch,0)*vec_foot;

FR_point4_xyz = [FR_point4_xyz(2);FR_point4_xyz(1);FR_point4_xyz(3);1];

FR_point4_pixels = FRCameraMatrix(1:2,:) * FR_point4_xyz./repmat(FRCameraMatrix(3,:) * FR_point4_xyz,2,1);

% h4 Equation
h11_eqn = FR_point4_pixels;
matlabFunction(h11_eqn,'file','h11_eqn');
% h11 Jacobian
H11_matrix_eq=jacobian(h11_eqn,X);
matlabFunction(H11_matrix_eq,'file','H11_matrix');
clear h11_eqn H11_matrix_eq

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%  back
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       BL POINT 1       %%%%%%         H12 right calf
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
BL_point1_xyz = vec_body_right + eulrot(0,RHipPitch,RHipYaw)*vec_thigh + ...
0.5*eulrot(0,RKneePitch,0)*vec_calf; 

BL_point1_xyz = [BL_point1_xyz(2);BL_point1_xyz(1);BL_point1_xyz(3);1];

BL_point1_pixels = FLCameraMatrix(1:2,:) * BL_point1_xyz./repmat(FLCameraMatrix(3,:) * BL_point1_xyz,2,1);

% h12 Equation
h12_eqn = BL_point1_pixels;
matlabFunction(h12_eqn,'file','h12_eqn');
% h12 Jacobian
H12_matrix_eq=jacobian(h12_eqn,X);
matlabFunction(H12_matrix_eq,'file','H12_matrix');
clear h12_eqn H12_matrix_eq

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       BL POINT 2       %%%%%%         H13 left calf
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
BL_point2_xyz = vec_body_left + eulrot(0,RHipPitch,RHipYaw)*vec_thigh + ...
0.5*eulrot(0,RKneePitch,0)*vec_calf; 

BL_point2_xyz = [BL_point2_xyz(2);BL_point2_xyz(1);BL_point2_xyz(3);1];

BL_point2_pixels = FLCameraMatrix(1:2,:) * BL_point2_xyz./repmat(FLCameraMatrix(3,:) * BL_point2_xyz,2,1);


% h13 Equation
h13_eqn = BL_point2_pixels;
matlabFunction(h13_eqn,'file','h13_eqn');
% h13 Jacobian
H13_matrix_eq=jacobian(h13_eqn,X);
matlabFunction(H13_matrix_eq,'file','H13_matrix');
clear h13_eqn H13_matrix_eq

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       BL POINT 3       %%%%%%         H14 right heel
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
BL_point3_xyz = vec_body_right + eulrot(0,RHipPitch,RHipYaw)*vec_thigh + ...
eulrot(0,RKneePitch,0)*(vec_calf + [0;0;15]/1000); 

BL_point3_xyz = [BL_point3_xyz(2);BL_point3_xyz(1);BL_point3_xyz(3);1];

BL_point3_pixels = FLCameraMatrix(1:2,:) * BL_point3_xyz./repmat(FLCameraMatrix(3,:) * BL_point3_xyz,2,1);


% h14 Equation
h14_eqn = BL_point3_pixels;
matlabFunction(h14_eqn,'file','h14_eqn');
% h14 Jacobian
H14_matrix_eq=jacobian(h14_eqn,X);
matlabFunction(H14_matrix_eq,'file','H14_matrix');
clear h14_eqn H14_matrix_eq

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       BL POINT 4       %%%%%%         H15 left heel
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
BL_point4_xyz = vec_body_left + eulrot(0,RHipPitch,RHipYaw)*vec_thigh + ...
eulrot(0,RKneePitch,0)*(vec_calf + [0;0;15]); 

BL_point4_xyz = [BL_point4_xyz(2);BL_point4_xyz(1);BL_point4_xyz(3);1];

BL_point4_pixels = FLCameraMatrix(1:2,:) * BL_point4_xyz./repmat(FLCameraMatrix(3,:) * BL_point4_xyz,2,1);


% h15 Equation
h15_eqn = BL_point4_pixels;
matlabFunction(h15_eqn,'file','h15_eqn');
% h15 Jacobian
H15_matrix_eq=jacobian(h15_eqn,X);
matlabFunction(H15_matrix_eq,'file','H15_matrix');
clear h15_eqn H15_matrix_eq

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       BR POINT 1       %%%%%%         H16 right calf
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
BR_point1_xyz = vec_body_right + eulrot(0,RHipPitch,RHipYaw)*vec_thigh + ...
0.5*eulrot(0,RKneePitch,0)*vec_calf; 

BR_point1_xyz = [BR_point1_xyz(2);BR_point1_xyz(1);BR_point1_xyz(3);1];

BR_point1_pixels = FLCameraMatrix(1:2,:) * BR_point1_xyz./repmat(FLCameraMatrix(3,:) * BR_point1_xyz,2,1);


% h16 Equation
h16_eqn = [BR_point1_pixels];
matlabFunction(h16_eqn,'file','h16_eqn');
% h16 Jacobian
H16_matrix_eq=jacobian(h16_eqn,X);
matlabFunction(H16_matrix_eq,'file','H16_matrix');
clear h16_eqn H16_matrix_eq

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       BR POINT 2       %%%%%%         H17 left calf
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
BR_point2_xyz = vec_body_left + eulrot(0,RHipPitch,RHipYaw)*vec_thigh + ...
0.5*eulrot(0,RKneePitch,0)*vec_calf;

BR_point2_xyz = [BR_point2_xyz(2);BR_point2_xyz(1);BR_point2_xyz(3);1];

BR_point2_pixels = FLCameraMatrix(1:2,:) * BR_point2_xyz./repmat(FLCameraMatrix(3,:) * BR_point2_xyz,2,1);


% h17 Equation
h17_eqn = [BR_point2_pixels];
matlabFunction(h17_eqn,'file','h17_eqn');
% h17 Jacobian
H17_matrix_eq=jacobian(h17_eqn,X);
matlabFunction(H17_matrix_eq,'file','H17_matrix');
clear h17_eqn H17_matrix_eq

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       BR POINT 3       %%%%%%         H18 right heel
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
BR_point3_xyz = vec_body_right + eulrot(0,RHipPitch,RHipYaw)*vec_thigh + ...
eulrot(0,RKneePitch,0)*(vec_calf + [0;0;15]/1000);

BR_point3_xyz = [BR_point3_xyz(2);BR_point3_xyz(1);BR_point3_xyz(3);1];

BR_point3_pixels = FLCameraMatrix(1:2,:) * BR_point3_xyz./repmat(FLCameraMatrix(3,:) * BR_point3_xyz,2,1);


% h18 Equation
h18_eqn = BR_point3_pixels;
matlabFunction(h18_eqn,'file','h18_eqn');
% h18 Jacobian
H18_matrix_eq=jacobian(h18_eqn,X);
matlabFunction(H18_matrix_eq,'file','H18_matrix');
clear h18_eqn H18_matrix_eq

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       BR POINT 4       %%%%%%         H19 left heel
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
BR_point4_xyz = vec_body_left + eulrot(0,RHipPitch,RHipYaw)*vec_thigh + ...
eulrot(0,RKneePitch,0)*(vec_calf + [0;0;15]/1000);

BR_point4_xyz = [BR_point4_xyz(2);BR_point4_xyz(1);BR_point4_xyz(3);1];

BR_point4_pixels = FLCameraMatrix(1:2,:) * BR_point4_xyz./repmat(FLCameraMatrix(3,:) * BR_point4_xyz,2,1);


% h19 Equation
h19_eqn = BR_point4_pixels;
matlabFunction(h19_eqn,'file','h19_eqn');
% h19 Jacobian
H19_matrix_eq=jacobian(h19_eqn,X);
matlabFunction(H19_matrix_eq,'file','H19_matrix');
clear h19_eqn H19_matrix_eq
