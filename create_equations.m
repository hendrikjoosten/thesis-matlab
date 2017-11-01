


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
X = [ bodyX; bodyY; bodyZ; bodyRoll; bodyPitch; bodyYaw;...
 LHipPitch; LHipYaw; LKneePitch; LAnklePitch;...
 RHipPitch; RHipYaw; RKneePitch; RAnklePitch;...
 d_bodyX; d_bodyY; d_bodyZ; d_bodyRoll; d_bodyPitch; d_bodyYaw;...
 d_LHipPitch; d_LHipYaw; d_LKneePitch; d_LAnklePitch;...
 d_RHipPitch; d_RHipYaw; d_RKneePitch; d_RAnklePitch;...
 dd_bodyX; dd_bodyY; dd_bodyZ; dd_bodyRoll; dd_bodyPitch; dd_bodyYaw;...
 dd_LHipPitch; dd_LHipYaw; dd_LKneePitch; dd_LAnklePitch;...
 dd_RHipPitch; dd_RHipYaw; dd_RKneePitch; dd_RAnklePitch];

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
sT = 0.01;

predict_eq_linAcc = vec_acc;                    % assume constant lin accel
predict_eq_linVel = vec_vel + sT*vec_acc;       % linear velocity
predict_eq_linPos = vec_pos + sT*vec_pos;       % linear position

predict_eq_angAcc = vec_domega;                 % assume constant ang accel
predict_eq_angVel = vec_omega + sT*vec_domega;  % angular velocity
predict_eq_angPos = vec_iomega + sT*vec_omega;  % angular position

predict_eq_legsAcc = vec_ddlegs;                %leg angles constant accel
predict_eq_legsVel = vec_dlegs + sT*vec_ddlegs; %leg velocity
predict_eq_legsPos = vec_legs + sT*vec_dlegs;   %leg position angles

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

%sysm here to create a function for the body
predict_eq = simplify(predict_eq);
matlabFunction(predict_eq,'file','state_predict_function');
%F matrix
Fmatrix_eq=jacobian(predict_eq,X);
matlabFunction(Fmatrix_eq,'file','Fmatrix_function');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%measurement needed syms
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%magnotometer
syms mx my mz
vec_mag = [mx; my; mz];
%barometer
syms bar
%gps
syms gpsx gpsy
vec_gps_pos = [gpsx; gpsy];
syms pgsvel gpshead
vec_gps_vel = [pgsvel; gpshead];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%h1 IMU
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
AccSensor = vec_acc;            
GyroSensor = vec_omega;
MagSensor = vec_mag;
% h1 Equation
h1_eqn = [AccSensor; GyroSensor; MagSensor];
h1_eqn = simplify(h1_eqn);
matlabFunction(h1_eqn,'file','h1_eqn');
% h1 Jacobian
H1_matrix_eq=jacobian(h1_eqn,X);
matlabFunction(H1_matrix_eq,'file','H1_matrix');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%h2 barometer
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
PressSens = 1013.25*(1-bodyZ/44307.7)^5.25;
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
%front left
FLCameraMatrix = cameraMatrix(FrontStereoSession.calibrationSession.CameraParameters.CameraParameters1,0,0);
%back right
BRCameraMatrix = cameraMatrix(BackStereoSession.calibrationSession.CameraParameters.CameraParameters1,0,0);
%back left
BLCamesraMatrix = cameraMatrix(BackStereoSession.calibrationSession.CameraParameters.CameraParameters2,...
BackStereoSession.calibrationSession.CameraParameters.RotationOfCamera2,...
BackStereoSession.calibrationSession.CameraParameters.TranslationOfCamera2);

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
FL_point1_xyz = vec_body_right + eul2rotm([0,RHipPitch,RHipYaw])*vec_thigh + [40;0;0];
FL_point1_pixels = FLCameraMatrix * FL_point1_xyz;
% h4 Equation
h4_eqn = [FL_point1_pixels];
matlabFunction(h4_eqn,'file','h4_eqn');
% h4 Jacobian
H4_matrix_eq=jacobian(h4_eqn,X);
matlabFunction(H4_matrix_eq,'file','H4_matrix');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       FL POINT 2       %%%%%%         H5
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
FL_point2_xyz = vec_body_left + eul2rotm([0,LHipPitch,LHipYaw])*vec_thigh + [40;0;0]; 
FL_point2_pixels = FLCameraMatrix * FL_point2_xyz; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       FL POINT 3       %%%%%%         H6
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
FL_point3_xyz = FL_point1_xyz + eul2rotm([0,RKneePitch,0])*vec_calf + eul2rotm([0,RAnklePitch,0])*vec_calf + [40;0;0]; 
FL_point3_pixels = FLCameraMatrix * FL_point3_xyz; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       FL POINT 4       %%%%%%         H7
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
FL_point4_xyz = FL_point2_xyz + eul2rotm([0,LKneePitch,0])*vec_calf + eul2rotm([0,LAnklePitch,0])*vec_foot + [40;0;0]; 
FL_point4_pixels = FLCameraMatrix * FL_point4_xyz;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       FR POINT 1       %%%%%%         H8
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
FR_point1_xyz = vec_body_right + eul2rotm([0,RHipPitch,RHipYaw])*vec_thigh;
FR_point1_pixels = FRCameraMatrix * FR_point1_xyz;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       FR POINT 2       %%%%%%         H9
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
FR_point2_xyz = vec_body_left + eul2rotm([0,LHipPitch,LHipYaw])*vec_thigh;
FR_point2_pixels = FRCameraMatrix * FR_point2_xyz;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       FR POINT 3       %%%%%%         H10
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
FR_point3_xyz = FR_point1_xyz + eul2rotm([0,RKneePitch,0])*vec_calf + eul2rotm([0,RAnklePitch,0])*vec_calf;
FR_point3_pixels = FRCameraMatrix * FR_point3_xyz;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       FR POINT 4       %%%%%%         H11
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
FR_point4_xyz = FR_point2_xyz + eul2rotm([0,LKneePitch,0])*vec_calf + eul2rotm([0,LAnklePitch,0])*vec_foot;
FR_point4_pixels = FRCameraMatrix * FR_point4_xyz;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       BL POINT 1       %%%%%%         H12
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
BL_point1_xyz = vec_body_right + eul2rotm([0,RHipPitch,RHipYaw])*vec_thigh + ...
0.5*eul2rotm([0,RKneePitch,0])*vec_calf + [40;0;0]; 
BL_point1_pixels = BLCameraMatrix * BL_point1_xyz;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       BL POINT 2       %%%%%%         H13
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
BL_point2_xyz = vec_body_left + eul2rotm([0,RHipPitch,RHipYaw])*vec_thigh + ...
0.5*eul2rotm([0,RKneePitch,0])*vec_calf + [40;0;0]; 
BL_point2_pixels = BLCameraMatrix * BL_point2_xyz; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       BL POINT 3       %%%%%%         H14
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
BL_point3_xyz = vec_body_right + eul2rotm([0,RHipPitch,RHipYaw])*vec_thigh + ...
eul2rotm([0,RKneePitch,0])*(vec_calf + [0;0;15]) + [40;0;0]; 
BL_point3_pixels = BLCameraMatrix * BL_point3_xyz;  

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       BL POINT 4       %%%%%%         H15
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
BL_point4_xyz = vec_body_left + eul2rotm([0,RHipPitch,RHipYaw])*vec_thigh + ...
eul2rotm([0,RKneePitch,0])*(vec_calf + [0;0;15]) + [40;0;0]; 
BL_point4_pixels = BLCameraMatrix * BL_point4_xyz;  


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       BR POINT 1       %%%%%%         H16
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
BR_point1_xyz = vec_body_right + eul2rotm([0,RHipPitch,RHipYaw])*vec_thigh + ...
0.5*eul2rotm([0,RKneePitch,0])*vec_calf; 
BR_point1_pixels = BRCameraMatrix * BR_point1_xyz;  

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       BR POINT 2       %%%%%%         H17
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
BR_point2_xyz = vec_body_left + eul2rotm([0,RHipPitch,RHipYaw])*vec_thigh + ...
0.5*eul2rotm([0,RKneePitch,0])*vec_calf;
BR_point2_pixels = BRCameraMatrix * BR_point2_xyz; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       BR POINT 3       %%%%%%         H18
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
BR_point3_xyz = vec_body_right + eul2rotm([0,RHipPitch,RHipYaw])*vec_thigh + ...
eul2rotm([0,RKneePitch,0])*(vec_calf + [0;0;15]);
BR_point3_pixels = BRCameraMatrix * BR_point3_xyz; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%       BR POINT 4       %%%%%%         H19
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
BR_point4_xyz = vec_body_left + eul2rotm([0,RHipPitch,RHipYaw])*vec_thigh + ...
eul2rotm([0,RKneePitch,0])*(vec_calf + [0;0;15]);
BR_point4_pixels = BRCameraMatrix * BR_point4_xyz;









