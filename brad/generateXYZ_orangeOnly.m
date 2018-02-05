function [XYZ, boolXYZ] = generateXYZ_orangeOnly(film1, start, stop)
load('/Users/bradstocks/Documents/MATLAB/Thesis/calibration_struct.mat')

XYZ = [];
boolXYZ = [];
%read in films
video1 = VideoReader(film1); 

for i = start:stop
    
    %read in individual frames
    image1 = read(video1, i);

    [orangeFinal, orangeFound] = generateOrangeXYZ_oneCamera(image1);
    
    if(orangeFound == 0) 
        orangeFinal = [0 0]; 
    end
    
    XYZ = [XYZ; [i orangeFinal]];
    boolXYZ = [boolXYZ; orangeFound];
end
end