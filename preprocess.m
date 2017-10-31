% IMU
IMU_Data = importdata('IMU.csv',',');
% 7 - LINEAR ACCELERATION X	
% 8 - LINEAR ACCELERATION Y	
% 9 - LINEAR ACCELERATION Z	
% 10 - GYROSCOPE X	
% 11 - GYROSCOPE Y	
% 12 - GYROSCOPE Z	
% 13 - MAGNETIC FIELD X	
% 14 - MAGNETIC FIELD Y	
% 15 - MAGNETIC FIELD Z		
% 19 - ATMOSPHERIC PRESSURE (Pascal)	
% 20 - LOCATION Latitude	
% 21 - LOCATION Longitude	
% 22 - LOCATION Speed	
% 23 - LOCATION ORIENTATION

%populate vectors in body frame
body_accel_x = IMU_Data.data(:,9);      % Z1
body_accel_y = -IMU_Data.data(:,8);     % Z1 
body_accel_z = -IMU_Data.data(:,7);     % Z1
body_gyro_x = IMU_Data.data(:,12);      % Z1
body_gyro_y = -IMU_Data.data(:,11);     % Z1 
body_gyro_z = -IMU_Data.data(:,10);     % Z1
body_mag_x = IMU_Data.data(:,15);       % Z1 
body_mag_y = -IMU_Data.data(:,14);      % Z1 
body_mag_z = -IMU_Data.data(:,13);      % Z1 

body_barometer = IMU_Data.data(:,19);   % Z2

gps_lat = IMU_Data.data(:,20);          % Z3 
gps_long = IMU_Data.data(:,21);         % Z3 
gps_speed = IMU_Data.data(:,22);        % Z3 
gps_head = IMU_Data.data(:,23);         % Z3

% IMU - Z1
% always availible
z01Avail = ones(length(body_accel_x),1);

% Pressure - Z2
z02Avail = zeros(length(body_barometer),1);
z02Avail(find((abs(diff(body_barometer)))>0)+1) = 1;

% GPS - Z3
z03Avail = zeros(length(gps_speed),1);
z03Avail(find((abs(diff(gps_speed)))>0)+1) = 1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Camera data points
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

% points and numbering references
BL_Data = importdata('BL.csv',',');
BR_Data = importdata('BR.csv',',');
FL_Data = importdata('FL.csv',',');
FR_Data = importdata('FR.csv',',');
%extracing x y pixel data
fl1 = FL_Data.data(:,1:2);      %Z4
fl2 = FL_Data.data(:,3:4);      %Z5
fl3 = FL_Data.data(:,5:6);      %Z6
fl4 = FL_Data.data(:,7:8);      %Z7
fr1 = FR_Data.data(:,1:2);      %Z8
fr2 = FR_Data.data(:,3:4);      %Z9
fr3 = FR_Data.data(:,5:6);      %Z10
fr4 = FR_Data.data(:,7:8);      %Z11
bl1 = BL_Data.data(:,1:2);      %Z12
bl2 = BL_Data.data(:,3:4);      %Z13
bl3 = BL_Data.data(:,5:6);      %Z14
bl4 = BL_Data.data(:,7:8);      %Z15
br1 = BR_Data.data(:,1:2);      %Z16
br2 = BR_Data.data(:,3:4);      %Z17
br3 = BR_Data.data(:,5:6);      %Z18
br4 = BR_Data.data(:,7:8);      %Z19

% Z4
z04Avail = zeros(length(fl1),1);
z04Avail(find(fl1(:,1))) = 1;

% Z5
z05Avail = zeros(length(fl1),1);
z05Avail(find(fl2(:,1))) = 1;

% Z6
z06Avail = zeros(length(fl1),1);
z06Avail(find(fl3(:,1))) = 1;

% Z7
z07Avail = zeros(length(fl1),1);
z07Avail(find(fl4(:,1))) = 1;

% Z8
z08Avail = zeros(length(fl1),1);
z08Avail(find(fr1(:,1))) = 1;

% Z9
z09Avail = zeros(length(fl1),1);
z09Avail(find(fr2(:,1))) = 1;

% Z10
z10Avail = zeros(length(fl1),1);
z10Avail(find(fr3(:,1))) = 1;

% Z11
z11Avail = zeros(length(fl1),1);
z11Avail(find(fr4(:,1))) = 1;

% Z12
z12Avail = zeros(length(fl1),1);
z12Avail(find(bl1(:,1))) = 1;

% Z13
z13Avail = zeros(length(fl1),1);
z13Avail(find(bl2(:,1))) = 1;

% Z14
z14Avail = zeros(length(fl1),1);
z14Avail(find(bl3(:,1))) = 1;

% Z15
z15Avail = zeros(length(fl1),1);
z15Avail(find(bl4(:,1))) = 1;

% Z16
z16Avail = zeros(length(fl1),1);
z16Avail(find(br1(:,1))) = 1;

% Z17
z17Avail = zeros(length(fl1),1);
z17Avail(find(br2(:,1))) = 1;

% Z18
z18Avail = zeros(length(fl1),1);
z18Avail(find(br3(:,1))) = 1;

% Z19
z19Avail = zeros(length(fl1),1);
z19Avail(find(br4(:,1))) = 1;

