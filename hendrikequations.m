%%mesurement equations (for each camera per dot
% camera Front Right (FR) FRONT camera 2 in calib session
% camera Front Left (FL) FRONT camera 1 in cailb session
% camera Back Right (BR) BACK camera 1 in calib session
% camera Back Left (BL) Back camera 2 in calib session

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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%FRONT RIGHT CAMERA AS REFERENCE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%loading the camera models as stereo calibration sessions...
FrontStereoSession = load('FRONTcalibrationSession.mat');
BackStereoSession = load('BACKcalibrationSession.mat');

%%defining the camera matrices.
%front right
FRCameraMatrix = cameraMatrix(FrontStereoSession.calibrationSession.CameraParameters.CameraParameters2,
FrontStereoSession.calibrationSession.CameraParameters.RotationOfCamera2,
FrontStereoSession.calibrationSession.CameraParameters.TranslationOfCamera2);
%front left
FLCameraMatrix = cameraMatrix(FrontStereoSession.calibrationSession.CameraParameters.CameraParameters1,0,0);
%back right
BRCameraMatrix = cameraMatrix(BackStereoSession.calibrationSession.CameraParameters.CameraParameters1,0,0);
%back left
BLCamesraMatrix = cameraMatrix((BackStereoSession.calibrationSession.CameraParameters.CameraParameters2,
BackStereoSession.calibrationSession.CameraParameters.RotationOfCamera2,
BackStereoSession.calibrationSession.CameraParameters.TranslationOfCamera2);


%%Finding the measurement equations froma single camera
%%Relating states to the pixels...

%start by defining the needed states and the different length vectors

vec_thigh = [0;0;400];
vec_calf = [0;0;400];
vec_foot = [200;0;0];
vec_body_right = [-10;-100;100];
vec_body_left = [-10;100;100];

%%states
% right_ankle_pitch 
% right_knee_pitch
% right_hip_pitch
% right_hip_yaw
% left_ankle_pitch
% left_knee_pitch
% left_hip_pitch
% left_hip_yaw



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%ANGS TO POSITIONS 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%FR camera seeing point 1 (right knee)
FR_point1_xyz = vec_body_right + eul2rotm([0,right_hip_pitch,right_hip_yaw])*vec_thigh;
%FR camera seeing point 2 (left knee)
FR_point2_xyz = vec_body_left + eul2rotm([0,left_hip_pitch,left_hip_yaw])*vec_thigh;
%FR camera seeing point 3 (right foot)
FR_point3_xyz = FR_point1_xyz + eul2rotm([0,right_knee_pitch,0])*vec_calf + eul2rotm([0,right_ankle_pitch,0])*vec_calf;
%FR camera seeing point 4 (left foot)
FR_point4_xyz = FR_point2_xyz + eul2rotm([0,left_knee_pitch,0])*vec_calf + eul2rotm([0,left_ankle_pitch,0])*vec_foot;

%Measurement equations for the left camera 
FL_point1_xyz = FR_point1_xyz + [40;0;0]; %cameratranslation parameter
FL_point2_xyz = FR_point2_xyz + [40;0;0]; %cameratranslation parameter
FL_point3_xyz = FR_point3_xyz + [40;0;0]; %cameratranslation parameter
FL_point4_xyz = FR_point4_xyz + [40;0;0]; %cameratranslation parameter

%BR camera seeing point 1 (right calf)
BR_point1_xyz = vec_body_right + eul2rotm([0,right_hip_pitch,right_hip_yaw])*vec_thigh + 
0.5*eul2rotm([0,right_knee_pitch,0])*vec_calf; 
%BR camera seeing point 2 (left calf)
BR_point2_xyz = vec_body_right + eul2rotm([0,right_hip_pitch,right_hip_yaw])*vec_thigh + 
0.5*eul2rotm([0,right_knee_pitch,0])*vec_calf;
%BR camera seeing point 3 (right heel)
BR_point3_xyz = vec_body_right + eul2rotm([0,right_hip_pitch,right_hip_yaw])*vec_thigh + 
eul2rotm([0,right_knee_pitch,0])*(vec_calf + [0;0;15]);
%BR camera seeing point 4 (left heel)
BR_point4_xyz = vec_body_right + eul2rotm([0,right_hip_pitch,right_hip_yaw])*vec_thigh + 
eul2rotm([0,right_knee_pitch,0])*(vec_calf + [0;0;15]);

%Measurement equations for the left camera 
BL_point1_xyz = BR_point1_xyz + [40;0;0]; %cameratranslation parameter
BL_point2_xyz = BR_point2_xyz + [40;0;0]; %cameratranslation parameter
BL_point3_xyz = BR_point3_xyz + [40;0;0]; %cameratranslation parameter
BL_point4_xyz = BR_point4_xyz + [40;0;0]; %cameratranslation parameter


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%POSITIONS TO PIXELS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
FR_point1_pixels = FRCameraMatrix * FR_point1_xyz;             %1
FR_point2_pixels = FRCameraMatrix * FR_point2_xyz;             %2
FR_point3_pixels = FRCameraMatrix * FR_point3_xyz;             %3
FR_point4_pixels = FRCameraMatrix * FR_point4_xyz;             %4

FL_point1_pixels = FLCameraMatrix * FL_point1_xyz;             %5
FL_point2_pixels = FLCameraMatrix * FL_point2_xyz;             %6
FL_point3_pixels = FLCameraMatrix * FL_point3_xyz;             %7
FL_point4_pixels = FLCameraMatrix * FL_point4_xyz;             %8

BR_point1_pixels = BRCameraMatrix * BR_point1_xyz;             %9
BR_point2_pixels = BRCameraMatrix * BR_point2_xyz;             %10
BR_point3_pixels = BRCameraMatrix * BR_point3_xyz;             %11
BR_point4_pixels = BRCameraMatrix * BR_point4_xyz;             %12

BL_point1_pixels = BLCameraMatrix * BL_point1_xyz;             %13
BL_point2_pixels = BLCameraMatrix * BL_point2_xyz;             %14
BL_point3_pixels = BLCameraMatrix * BL_point3_xyz;             %15
BL_point4_pixels = BLCameraMatrix * BL_point4_xyz;             %16


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%IMU
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%mesurement equations 
%wanna go from states to meausurements

















