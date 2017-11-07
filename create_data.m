%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Importing data csv
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

IMU_Data = importdata('IMU.csv',',');
BL_Data = importdata('BL.csv',',');
BR_Data = importdata('BR.csv',',');
FL_Data = importdata('FL.csv',',');
FR_Data = importdata('FR.csv',',');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Headings of IMU dataset
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 7 - LINEAR ACCELERATION X	
% 8 - LINEAR ACCELERATION Y	
% 9 - LINEAR ACCELERATION Z	
% 10 - GYROSCOPE X	
% 11 - GYROSCOPE Y	
% 12 - GYROSCOPE Z	
% 13 - MAGNETIC FIELD X	
% 14 - MAGNETIC FIELD Y	
% 15 - MAGNETIC FIELD Z		
% 19 - ATMOSPHERIC PRESSURE	
% 20 - LOCATION Latitude	
% 21 - LOCATION Longitude	
% 22 - LOCATION Speed	
% 23 - LOCATION ORIENTATION

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%populate vectors in FROM IMU FRAME TO BODY FRAME
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

body_accel_x = IMU_Data.data(:,9);           % Z1  m/s2
body_accel_y = -IMU_Data.data(:,8);          % Z1  m/s2
body_accel_z = -IMU_Data.data(:,7);          % Z1  m/s2
body_gyro_x = IMU_Data.data(:,12);           % Z1  rad/s
body_gyro_y = -IMU_Data.data(:,11);          % Z1  rad/s
body_gyro_z = -IMU_Data.data(:,10);          % Z1  rad/s
body_mag_x = IMU_Data.data(:,15);            % Z1  
body_mag_y = -IMU_Data.data(:,14);           % Z1 
body_mag_z = -IMU_Data.data(:,13);           % Z1 

body_barometer = IMU_Data.data(:,19);        % Z2
body_gps_lat = IMU_Data.data(:,20);          % Z3 
body_gps_long = IMU_Data.data(:,21);         % Z3
body_gps_speed = IMU_Data.data(:,22);        % Z3 
body_gps_head = IMU_Data.data(:,23);         % Z3

%http://msi.nga.mil/MSISiteContent/StaticFiles/Calculators/degree.html

body_gps_pos = [(body_gps_lat-body_gps_lat(1))*110922,(body_gps_long-body_gps_long(1))*92423];
body_gps_vel = [body_gps_speed.*cosd(body_gps_head), body_gps_speed.*sind(body_gps_head)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Camera data points
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% +----+--------+-------+-------------+
% | Z  | Camera | Point | Description |
% +----+--------+-------+-------------+
% | 4  | FL     | 1     | right knee  |
% +----+--------+-------+-------------+
% | 5  | FL     | 2     | left knee   |
% +----+--------+-------+-------------+
% | 6  | FL     | 3     | right toe   |
% +----+--------+-------+-------------+
% | 7  | FL     | 4     | left toe    |
% +----+--------+-------+-------------+
% | 8  | FR     | 1     | right knee  |
% +----+--------+-------+-------------+
% | 9  | FR     | 2     | left knee   |
% +----+--------+-------+-------------+
% | 10 | FR     | 3     | right toe   |
% +----+--------+-------+-------------+
% | 11 | FR     | 4     | left toe    |
% +----+--------+-------+-------------+
% | 12 | BL     | 1     | right calf  |
% +----+--------+-------+-------------+
% | 13 | BL     | 2     | left calf   |
% +----+--------+-------+-------------+
% | 14 | BL     | 3     | right heel  |
% +----+--------+-------+-------------+
% | 15 | BL     | 4     | left heel   |
% +----+--------+-------+-------------+
% | 16 | BR     | 1     | right calf  |
% +----+--------+-------+-------------+
% | 17 | BR     | 2     | left calf   |
% +----+--------+-------+-------------+
% | 18 | BR     | 3     | right heel  |
% +----+--------+-------+-------------+
% | 19 | BR     | 4     | left heel   |
% +----+--------+-------+-------------+

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Importing video data values from dldvt from bot to top conversion
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%FRONT LEFT
fl1 = FL_Data.data(:,1:2);      %Z4 - See Table
fl2 = FL_Data.data(:,3:4);      %Z5 - See Table
fl3 = FL_Data.data(:,5:6);      %Z6 - See Table
fl4 = FL_Data.data(:,7:8);      %Z7 - See Table
%FRONT RIGHT
fr1 = FR_Data.data(:,1:2);      %Z8 - See Table
fr2 = FR_Data.data(:,3:4);      %Z9 - See Table
fr3 = FR_Data.data(:,5:6);      %Z10 - See Table
fr4 = FR_Data.data(:,7:8);      %Z11 - See Table
%BACK LEFT
bl1 = BL_Data.data(:,1:2);      %Z12 - See Table
bl2 = BL_Data.data(:,3:4);      %Z13 - See Table
bl3 = BL_Data.data(:,5:6);      %Z14 - See Table
bl4 = BL_Data.data(:,7:8);      %Z15 - See Table
%BACK RIGHT
br1 = BR_Data.data(:,1:2);      %Z16 - See Table
br2 = BR_Data.data(:,3:4);      %Z17 - See Table
br3 = BR_Data.data(:,5:6);      %Z18 - See Table
br4 = BR_Data.data(:,7:8);      %Z19 - See Table


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generating availibility vectors for EKF
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Z1 IMU
z01Avail = ones(length(body_accel_x),1);

% Z2 - Pressure
z02Avail = zeros(length(body_barometer),1);
z02Avail(find((abs(diff(body_barometer)))>0)+1) = 1;

% Z3 - GPS
z03Avail = zeros(length(body_gps_speed),1);
z03Avail(find((abs(diff(body_gps_speed)))>0)+1) = 1;

% Z4 - See Table
z04Avail = zeros(length(fl1),1);
z04Avail(find(fl1(:,1))) = 1;

% Z5 - See Table
z05Avail = zeros(length(fl1),1);
z05Avail(find(fl2(:,1))) = 1;

% Z6 - See Table
z06Avail = zeros(length(fl1),1);
z06Avail(find(fl3(:,1))) = 1;

% Z7 - See Table
z07Avail = zeros(length(fl1),1);
z07Avail(find(fl4(:,1))) = 1;

% Z8 - See Table
z08Avail = zeros(length(fl1),1);
z08Avail(find(fr1(:,1))) = 1;

% Z9 - See Table
z09Avail = zeros(length(fl1),1);
z09Avail(find(fr2(:,1))) = 1;

% Z10 - See Table
z10Avail = zeros(length(fl1),1);
z10Avail(find(fr3(:,1))) = 1;

% Z11 - See Table
z11Avail = zeros(length(fl1),1);
z11Avail(find(fr4(:,1))) = 1;

% Z12 - See Table
z12Avail = zeros(length(fl1),1);
z12Avail(find(bl1(:,1))) = 1;

% Z13 - See Table
z13Avail = zeros(length(fl1),1);
z13Avail(find(bl2(:,1))) = 1;

% Z14 - See Table
z14Avail = zeros(length(fl1),1);
z14Avail(find(bl3(:,1))) = 1;

% Z15 - See Table
z15Avail = zeros(length(fl1),1);
z15Avail(find(bl4(:,1))) = 1;

% Z16 - See Table
z16Avail = zeros(length(fl1),1);
z16Avail(find(br1(:,1))) = 1;

% Z17 - See Table
z17Avail = zeros(length(fl1),1);
z17Avail(find(br2(:,1))) = 1;

% Z18 - See Table
z18Avail = zeros(length(fl1),1);
z18Avail(find(br3(:,1))) = 1;

% Z19 - See Table
z19Avail = zeros(length(fl1),1);
z19Avail(find(br4(:,1))) = 1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% rotating camera frames
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




fl1(:,1) = 1280 - fl1(:,1);      %change origin to matlab spec
fl2(:,1) = 1280 - fl2(:,1);      %change origin to matlab spec
fl3(:,1) = 1280 - fl3(:,1);      %change origin to matlab spec
fl4(:,1) = 1280 - fl4(:,1);      %change origin to matlab spec
fr1(:,1) = 1280 - fr1(:,1);      %change origin to matlab spec
fr2(:,1) = 1280 - fr2(:,1);      %change origin to matlab spec
fr3(:,1) = 1280 - fr3(:,1);      %change origin to matlab spec
fr4(:,1) = 1280 - fr4(:,1);      %change origin to matlab spec
bl1(:,2) = 720 - bl1(:,2);      %change origin to matlab spec
bl2(:,2) = 720 - bl2(:,2);      %change origin to matlab spec
bl3(:,2) = 720 - bl3(:,2);      %change origin to matlab spec
bl4(:,2) = 720 - bl4(:,2);      %change origin to matlab spec
br1(:,2) = 720 - br1(:,2);      %change origin to matlab spec
br2(:,2) = 720 - br2(:,2);      %change origin to matlab spec
br3(:,2) = 720 - br3(:,2);      %change origin to matlab spec
br4(:,2) = 720 - br4(:,2);      %change origin to matlab spec






