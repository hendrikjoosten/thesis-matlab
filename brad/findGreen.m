function greenBin = findGreen(frame)
    greenBin= zeros(720, 1280);
    sample1 = [142 154 80];
    sample2 = [182 215 52];
    sample3 = [137 149 77];
    sample4 = [173 175 108];
    sample5 = [162 166 85];
    sample6 = [179 182 106];
    angle_threshold = 5;
    mag_threshold = 50;
    doubleFrame = double(frame);
    normFrame = sqrt(sum(doubleFrame(:, :, :).^2, 3));
    magDiff1 = abs(normFrame - sqrt(sum((sample1).^2)));
    magDiff2 = abs(normFrame - sqrt(sum((sample2).^2)));
    magDiff3 = abs(normFrame - sqrt(sum((sample3).^2)));
    magDiff4 = abs(normFrame - sqrt(sum((sample4).^2)));
    magDiff5 = abs(normFrame - sqrt(sum((sample5).^2)));
    magDiff6 = abs(normFrame - sqrt(sum((sample6).^2)));
    innerProduct1 = sample1(1)*doubleFrame(:,:,1) + sample1(2)*doubleFrame(:,:,2) + sample1(3)*doubleFrame(:,:,3);
    innerProduct2 = sample2(1)*doubleFrame(:,:,1) + sample2(2)*doubleFrame(:,:,2) + sample2(3)*doubleFrame(:,:,3);
    innerProduct3 = sample3(1)*doubleFrame(:,:,1) + sample3(2)*doubleFrame(:,:,2) + sample3(3)*doubleFrame(:,:,3);
    innerProduct4 = sample4(1)*doubleFrame(:,:,1) + sample4(2)*doubleFrame(:,:,2) + sample4(3)*doubleFrame(:,:,3);
    innerProduct5 = sample5(1)*doubleFrame(:,:,1) + sample5(2)*doubleFrame(:,:,2) + sample5(3)*doubleFrame(:,:,3);
    innerProduct6 = sample6(1)*doubleFrame(:,:,1) + sample6(2)*doubleFrame(:,:,2) + sample6(3)*doubleFrame(:,:,3);
    angle1 = acosd(innerProduct1./(norm(sample1)*normFrame));
    angle2 = acosd(innerProduct2./(norm(sample2)*normFrame));
    angle3 = acosd(innerProduct3./(norm(sample3)*normFrame));
    angle4 = acosd(innerProduct4./(norm(sample4)*normFrame));
    angle5 = acosd(innerProduct5./(norm(sample5)*normFrame));
    angle6 = acosd(innerProduct6./(norm(sample6)*normFrame));
    greenBin((angle1<angle_threshold) .* (magDiff1 < mag_threshold) == 1)=250;
    greenBin((angle2<angle_threshold) .* (magDiff2 < mag_threshold) == 1)=250;
    greenBin(angle3<angle_threshold .* (magDiff3 < mag_threshold) == 1)=250;
    greenBin(angle4<angle_threshold .* (magDiff4 < mag_threshold) == 1)=250;
    greenBin(angle5<angle_threshold .* (magDiff5 < mag_threshold) == 1)=250;
    greenBin(angle6<angle_threshold .* (magDiff6 < mag_threshold) == 1)=250;
    
    %need to find largest connected region
    sedisk = strel('disk', 4);
    greenBin = imopen(greenBin, sedisk);
    return
end