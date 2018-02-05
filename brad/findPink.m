function pinkBin = findPink(frame)
    pinkBin= zeros(720, 1280);
    sample1 = [111 49 70];
    sample2 = [100 48 61];
    sample3 = [121 68 86];
    sample4 = [76 37 37];
    sample5 = [150 101 103];
    sample6 = [175 82 100];
    sample7 = [98 65 74];
    sample8 = [97 50 70];
    
    angle_threshold = 3;
    mag_threshold = 50;
    doubleFrame = double(frame);
    normFrame = sqrt(sum(doubleFrame(:, :, :).^2, 3));
    magDiff1 = abs(normFrame - sqrt(sum((sample1).^2)));
    magDiff2 = abs(normFrame - sqrt(sum((sample2).^2)));
    magDiff3 = abs(normFrame - sqrt(sum((sample3).^2)));
    magDiff4 = abs(normFrame - sqrt(sum((sample4).^2)));
    magDiff5 = abs(normFrame - sqrt(sum((sample5).^2)));
    magDiff6 = abs(normFrame - sqrt(sum((sample6).^2)));
    magDiff7 = abs(normFrame - sqrt(sum((sample7).^2)));
    magDiff8 = abs(normFrame - sqrt(sum((sample8).^2)));
    
    innerProduct1 = sample1(1)*doubleFrame(:,:,1) + sample1(2)*doubleFrame(:,:,2) + sample1(3)*doubleFrame(:,:,3);
    innerProduct2 = sample2(1)*doubleFrame(:,:,1) + sample2(2)*doubleFrame(:,:,2) + sample2(3)*doubleFrame(:,:,3);
    innerProduct3 = sample3(1)*doubleFrame(:,:,1) + sample3(2)*doubleFrame(:,:,2) + sample3(3)*doubleFrame(:,:,3);
    innerProduct4 = sample4(1)*doubleFrame(:,:,1) + sample4(2)*doubleFrame(:,:,2) + sample4(3)*doubleFrame(:,:,3);
    innerProduct5 = sample5(1)*doubleFrame(:,:,1) + sample5(2)*doubleFrame(:,:,2) + sample5(3)*doubleFrame(:,:,3);
    innerProduct6 = sample6(1)*doubleFrame(:,:,1) + sample6(2)*doubleFrame(:,:,2) + sample6(3)*doubleFrame(:,:,3);
    innerProduct7 = sample7(1)*doubleFrame(:,:,1) + sample7(2)*doubleFrame(:,:,2) + sample7(3)*doubleFrame(:,:,3);
    innerProduct8 = sample8(1)*doubleFrame(:,:,1) + sample8(2)*doubleFrame(:,:,2) + sample8(3)*doubleFrame(:,:,3);

    angle1 = abs(acosd(innerProduct1./(norm(sample1)*normFrame)));
    angle2 = abs(acosd(innerProduct2./(norm(sample2)*normFrame)));
    angle3 = abs(acosd(innerProduct3./(norm(sample3)*normFrame)));
    angle4 = acosd(innerProduct4./(norm(sample4)*normFrame));
    angle5 = acosd(innerProduct5./(norm(sample5)*normFrame));
    angle6 = acosd(innerProduct6./(norm(sample6)*normFrame));
    angle7 = acosd(innerProduct7./(norm(sample7)*normFrame));
    angle8 = acosd(innerProduct8./(norm(sample8)*normFrame));

    
    %[magDiff1(487, 620:630); magDiff2(487, 620:630); magDiff3(487, 620:630)]
    %[angle1(487, 620:630); angle2(487, 620:630); angle3(487, 620:630)]
    pinkBin((angle1<angle_threshold) .* (magDiff1 < mag_threshold) == 1)=250;
    pinkBin((angle2<angle_threshold) .* (magDiff2 < mag_threshold) == 1)=250;
    pinkBin((angle3<angle_threshold) .* (magDiff3 < mag_threshold) == 1)=250;
    pinkBin(angle4<angle_threshold .* (magDiff4 < mag_threshold) == 1)=250;
    pinkBin(angle5<angle_threshold .* (magDiff5 < mag_threshold) == 1)=250;
    pinkBin(angle6<angle_threshold .* (magDiff6 < mag_threshold) == 1)=250;
    pinkBin(angle7<angle_threshold .* (magDiff7 < mag_threshold) == 1)=250;
    pinkBin(angle8<angle_threshold .* (magDiff8 < mag_threshold) == 1)=250;
    
    %need to find largest connected region
    sedisk = strel('disk', 4);
    pinkBin = imopen(pinkBin, sedisk);
    return
end