function orangeBin = findOrange(frame)
    orangeBin= zeros(720, 1280);
    sample1 = [225 65 50];
    sample2 = [240 60 40];
    sample3 = [250 170 100];
    angle_threshold = 10;
    mag_threshold = 50;
    doubleFrame = double(frame);
    normFrame = sqrt(sum(doubleFrame(:, :, :).^2, 3));
    magDiff1 = abs(normFrame - sqrt(sum((sample1).^2)));
    
    magDiff2 = abs(normFrame - sqrt(sum((sample2).^2)));
    magDiff3 = abs(normFrame - sqrt(sum((sample3).^2)));
    innerProduct1 = sample1(1)*doubleFrame(:,:,1) + sample1(2)*doubleFrame(:,:,2) + sample1(3)*doubleFrame(:,:,3);
    innerProduct2 = sample2(1)*doubleFrame(:,:,1) + sample2(2)*doubleFrame(:,:,2) + sample2(3)*doubleFrame(:,:,3);
    innerProduct3 = sample3(1)*doubleFrame(:,:,1) + sample3(2)*doubleFrame(:,:,2) + sample3(3)*doubleFrame(:,:,3);
    angle1 = acosd(innerProduct1./(norm(sample1)*normFrame));
    angle2 = acosd(innerProduct2./(norm(sample2)*normFrame));
    angle3 = acosd(innerProduct3./(norm(sample3)*normFrame));
    orangeBin((magDiff1 < mag_threshold) .* (angle1<angle_threshold) == 1)=250;
    orangeBin((angle2<angle_threshold) .* (magDiff2 < mag_threshold) == 1)=250;
    %orangeBin((angle3<angle_threshold) .* (magDiff3 < mag_threshold) == 1)=250;
    
    %need to find largest connected region
    sedisk = strel('disk', 4);
   % orangeBin = imopen(orangeBin, sedisk);
    return
end