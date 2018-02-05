function [XYZ, boolXYZ] = generateXYZ(film1, film2, start, stop, offset)
load('/Users/bradstocks/Documents/MATLAB/Thesis/calibration_struct.mat')

XYZ = [];
boolXYZ = [];
%read in films
video1 = VideoReader(film1); 
video2 = VideoReader(film2);

cp1 = stereoParams.CameraParameters1;  cp2 = stereoParams.CameraParameters2;  
P1 = [stereoParams.CameraParameters1.IntrinsicMatrix' [0;0;0]];
Rt2 = [stereoParams.RotationOfCamera2' stereoParams.TranslationOfCamera2'];
P2 = stereoParams.CameraParameters2.IntrinsicMatrix'*Rt2;


for i = start:stop
    
    %read in individual frames
    image1 = read(video1, i);
    image2 = read(video2, i + offset); %offset for synchronisation
    figure
    subplot(2, 1, 1)
    imshow(image1);
    subplot(2, 1, 2)
    imshow(image2);

    [orangeFinal1, orangeFinal2, orangeFound1, orangeFound2] = generateOrangeXYZ(image1, image2);
    [greenFinal1, greenFinal2, greenFound1, greenFound2] = generateGreenXYZ(image1, image2);
    [pinkFinal1, pinkFinal2, pinkFound1, pinkFound2] = generatePinkXYZ(image1, image2);
    
    undistOrange1 = undistortPoints(orangeFinal1,cp1) ;
    undistOrange2 = undistortPoints(orangeFinal2,cp2);
    
    undistGreen1 = undistortPoints(greenFinal1,cp1); 
    undistGreen2 = undistortPoints(greenFinal2,cp2);
    %fGreen1 = pointsToWorld(cp1, eye(3), [0 0 1], undistGreen1)
    %fGreen2 = pointsToWorld(cp2, stereoParams.RotationOfCamera2, stereoParams.TranslationOfCamera2, undistGreen2)
    
    undistPink1 = undistortPoints(pinkFinal1,cp1); 
    undistPink2 = undistortPoints(pinkFinal2,cp2);
    
    if(orangeFound1 == 1 && orangeFound2 == 1) 
        orangeTriang = triangulate(undistOrange1, undistOrange2, P1', P2');
        orangeZ = 1;
    else
        orangeTriang = [0 0 0];
        orangeZ = 0;
    end
    
    if(greenFound1 == 1 && greenFound2 == 1) 
        greenTriang = triangulate(undistGreen1, undistGreen2, P1', P2');
        greenZ = 1;
    else
        greenTriang = [0 0 0];
        greenZ = 0;
    end
    
    if(pinkFound1 == 1 && pinkFound2 == 1) 
        pinkTriang = triangulate(undistPink1, undistPink2, P1', P2');
        pinkZ = 1;
    else
        pinkTriang = [0 0 0];
        pinkZ = 0;
    end
    
    XYZ = [XYZ; [undistOrange1, undistOrange2, orangeTriang];
        [undistGreen1, undistGreen2, greenTriang];
        [undistPink1, undistPink2, pinkTriang]];
    boolXYZ = [boolXYZ; [orangeFound1, orangeFound2, orangeZ];
        [greenFound1,greenFound2, greenZ];
        [pinkFound1, pinkFound2, pinkZ]]; 
    
end
end