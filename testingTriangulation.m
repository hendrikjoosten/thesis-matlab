clear;
clc;

load('FRONTcalibrationSession.mat');

matchedPoints1 = [202,611];
matchedPoints2 = [250,670];

worldPoints = triangulate(matchedPoints1,matchedPoints2,calibrationSession.CameraParameters);
worldPoints











