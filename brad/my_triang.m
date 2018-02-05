function [xyz] = my_triang(p1, p2)
load('/Users/bradstocks/Documents/MATLAB/Thesis/calibration_struct.mat');
%This could work if cameras are both upright, inconsistent for rotated
%(especially green)

%p1(1) = (p1(1) - 640)/8.5;
%p1(2) = (p1(2) - 360)/8.5;
p2(2) = 720-p2(2);
p1
p2


%p2 = [p2, 0];
%p2 = stereoParams.RotationOfCamera2*p2';

z= (4*50) / (abs((p1(1)-260-p2(2)))/8.5);

x = (p1(1) - 640)/8.5;
y = -(p1(2) - 360)/8.5;
%x = p1(1) * z/53.4;
%y = p1(2) * z/53.4;

xyz = [x, y, z];
end