% Auto-generated by stereoCalibrator app on 06-Sep-2017
%-------------------------------------------------------


% Define images to process
imageFileNames1 = {'D:\calibration\BLGPHSBROKEN\snaps\FR001.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR051.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR101.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR1051.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR1101.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR1151.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR1201.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR1251.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR1301.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR1351.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR1401.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR1451.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR1501.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR151.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR1551.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR1601.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR1651.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR1701.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR1751.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR1801.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR1851.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR1901.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR1951.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR2001.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR201.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR2051.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR2101.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR2151.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR2201.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR2251.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR2301.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR2351.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR2401.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR2451.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR2501.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR251.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR2601.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR2651.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR2701.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR2751.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR2801.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR2851.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR2901.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR2951.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR3001.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR301.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR3051.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR3101.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR3151.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR3201.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR3251.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR3301.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR3351.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR3401.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR3451.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR3501.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR351.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR3551.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR3601.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR3651.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR3701.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR3751.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR3801.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR3851.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR3901.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR3951.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR4001.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR401.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR4051.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR4101.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR4151.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR4201.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR4251.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR4301.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR4351.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR4401.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR4451.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR4501.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR4551.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR4601.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR4651.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR4701.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR4751.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR4801.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR4851.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR4901.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR4951.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR5001.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR5051.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR5101.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR5151.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR5201.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR5251.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR5301.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR5351.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR601.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR651.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR701.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR751.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR801.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR851.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR901.png',...
    'D:\calibration\BLGPHSBROKEN\snaps\FR951.png',...
    };
imageFileNames2 = {'D:\calibration\BRGPHS\snaps\FR001.png',...
    'D:\calibration\BRGPHS\snaps\FR051.png',...
    'D:\calibration\BRGPHS\snaps\FR101.png',...
    'D:\calibration\BRGPHS\snaps\FR1051.png',...
    'D:\calibration\BRGPHS\snaps\FR1101.png',...
    'D:\calibration\BRGPHS\snaps\FR1151.png',...
    'D:\calibration\BRGPHS\snaps\FR1201.png',...
    'D:\calibration\BRGPHS\snaps\FR1251.png',...
    'D:\calibration\BRGPHS\snaps\FR1301.png',...
    'D:\calibration\BRGPHS\snaps\FR1351.png',...
    'D:\calibration\BRGPHS\snaps\FR1401.png',...
    'D:\calibration\BRGPHS\snaps\FR1451.png',...
    'D:\calibration\BRGPHS\snaps\FR1501.png',...
    'D:\calibration\BRGPHS\snaps\FR151.png',...
    'D:\calibration\BRGPHS\snaps\FR1551.png',...
    'D:\calibration\BRGPHS\snaps\FR1601.png',...
    'D:\calibration\BRGPHS\snaps\FR1651.png',...
    'D:\calibration\BRGPHS\snaps\FR1701.png',...
    'D:\calibration\BRGPHS\snaps\FR1751.png',...
    'D:\calibration\BRGPHS\snaps\FR1801.png',...
    'D:\calibration\BRGPHS\snaps\FR1851.png',...
    'D:\calibration\BRGPHS\snaps\FR1901.png',...
    'D:\calibration\BRGPHS\snaps\FR1951.png',...
    'D:\calibration\BRGPHS\snaps\FR2001.png',...
    'D:\calibration\BRGPHS\snaps\FR201.png',...
    'D:\calibration\BRGPHS\snaps\FR2051.png',...
    'D:\calibration\BRGPHS\snaps\FR2101.png',...
    'D:\calibration\BRGPHS\snaps\FR2151.png',...
    'D:\calibration\BRGPHS\snaps\FR2201.png',...
    'D:\calibration\BRGPHS\snaps\FR2251.png',...
    'D:\calibration\BRGPHS\snaps\FR2301.png',...
    'D:\calibration\BRGPHS\snaps\FR2351.png',...
    'D:\calibration\BRGPHS\snaps\FR2401.png',...
    'D:\calibration\BRGPHS\snaps\FR2451.png',...
    'D:\calibration\BRGPHS\snaps\FR2501.png',...
    'D:\calibration\BRGPHS\snaps\FR251.png',...
    'D:\calibration\BRGPHS\snaps\FR2601.png',...
    'D:\calibration\BRGPHS\snaps\FR2651.png',...
    'D:\calibration\BRGPHS\snaps\FR2701.png',...
    'D:\calibration\BRGPHS\snaps\FR2751.png',...
    'D:\calibration\BRGPHS\snaps\FR2801.png',...
    'D:\calibration\BRGPHS\snaps\FR2851.png',...
    'D:\calibration\BRGPHS\snaps\FR2901.png',...
    'D:\calibration\BRGPHS\snaps\FR2951.png',...
    'D:\calibration\BRGPHS\snaps\FR3001.png',...
    'D:\calibration\BRGPHS\snaps\FR301.png',...
    'D:\calibration\BRGPHS\snaps\FR3051.png',...
    'D:\calibration\BRGPHS\snaps\FR3101.png',...
    'D:\calibration\BRGPHS\snaps\FR3151.png',...
    'D:\calibration\BRGPHS\snaps\FR3201.png',...
    'D:\calibration\BRGPHS\snaps\FR3251.png',...
    'D:\calibration\BRGPHS\snaps\FR3301.png',...
    'D:\calibration\BRGPHS\snaps\FR3351.png',...
    'D:\calibration\BRGPHS\snaps\FR3401.png',...
    'D:\calibration\BRGPHS\snaps\FR3451.png',...
    'D:\calibration\BRGPHS\snaps\FR3501.png',...
    'D:\calibration\BRGPHS\snaps\FR351.png',...
    'D:\calibration\BRGPHS\snaps\FR3551.png',...
    'D:\calibration\BRGPHS\snaps\FR3601.png',...
    'D:\calibration\BRGPHS\snaps\FR3651.png',...
    'D:\calibration\BRGPHS\snaps\FR3701.png',...
    'D:\calibration\BRGPHS\snaps\FR3751.png',...
    'D:\calibration\BRGPHS\snaps\FR3801.png',...
    'D:\calibration\BRGPHS\snaps\FR3851.png',...
    'D:\calibration\BRGPHS\snaps\FR3901.png',...
    'D:\calibration\BRGPHS\snaps\FR3951.png',...
    'D:\calibration\BRGPHS\snaps\FR4001.png',...
    'D:\calibration\BRGPHS\snaps\FR401.png',...
    'D:\calibration\BRGPHS\snaps\FR4051.png',...
    'D:\calibration\BRGPHS\snaps\FR4101.png',...
    'D:\calibration\BRGPHS\snaps\FR4151.png',...
    'D:\calibration\BRGPHS\snaps\FR4201.png',...
    'D:\calibration\BRGPHS\snaps\FR4251.png',...
    'D:\calibration\BRGPHS\snaps\FR4301.png',...
    'D:\calibration\BRGPHS\snaps\FR4351.png',...
    'D:\calibration\BRGPHS\snaps\FR4401.png',...
    'D:\calibration\BRGPHS\snaps\FR4451.png',...
    'D:\calibration\BRGPHS\snaps\FR4501.png',...
    'D:\calibration\BRGPHS\snaps\FR4551.png',...
    'D:\calibration\BRGPHS\snaps\FR4601.png',...
    'D:\calibration\BRGPHS\snaps\FR4651.png',...
    'D:\calibration\BRGPHS\snaps\FR4701.png',...
    'D:\calibration\BRGPHS\snaps\FR4751.png',...
    'D:\calibration\BRGPHS\snaps\FR4801.png',...
    'D:\calibration\BRGPHS\snaps\FR4851.png',...
    'D:\calibration\BRGPHS\snaps\FR4901.png',...
    'D:\calibration\BRGPHS\snaps\FR4951.png',...
    'D:\calibration\BRGPHS\snaps\FR5001.png',...
    'D:\calibration\BRGPHS\snaps\FR5051.png',...
    'D:\calibration\BRGPHS\snaps\FR5101.png',...
    'D:\calibration\BRGPHS\snaps\FR5151.png',...
    'D:\calibration\BRGPHS\snaps\FR5201.png',...
    'D:\calibration\BRGPHS\snaps\FR5251.png',...
    'D:\calibration\BRGPHS\snaps\FR5301.png',...
    'D:\calibration\BRGPHS\snaps\FR5351.png',...
    'D:\calibration\BRGPHS\snaps\FR601.png',...
    'D:\calibration\BRGPHS\snaps\FR651.png',...
    'D:\calibration\BRGPHS\snaps\FR701.png',...
    'D:\calibration\BRGPHS\snaps\FR751.png',...
    'D:\calibration\BRGPHS\snaps\FR801.png',...
    'D:\calibration\BRGPHS\snaps\FR851.png',...
    'D:\calibration\BRGPHS\snaps\FR901.png',...
    'D:\calibration\BRGPHS\snaps\FR951.png',...
    };

% Detect checkerboards in images
[imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(imageFileNames1, imageFileNames2);

% Generate world coordinates of the checkerboard keypoints
squareSize = 24;  % in units of 'mm'
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Calibrate the camera
[stereoParams, pairsUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
    'EstimateSkew', false, 'EstimateTangentialDistortion', false, ...
    'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'mm');

% View reprojection errors
h1=figure; showReprojectionErrors(stereoParams, 'BarGraph');

% Visualize pattern locations
h2=figure; showExtrinsics(stereoParams, 'CameraCentric');

% Display parameter estimation errors
displayErrors(estimationErrors, stereoParams);

% You can use the calibration data to rectify stereo images.
I1 = imread(imageFileNames1{1});
I2 = imread(imageFileNames2{1});
[J1, J2] = rectifyStereoImages(I1, I2, stereoParams);

% See additional examples of how to use the calibration data.  At the prompt type:
% showdemo('StereoCalibrationAndSceneReconstructionExample')
