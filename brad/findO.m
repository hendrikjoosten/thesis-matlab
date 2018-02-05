function orangeBin = findO(frame)
    %subplot(2, 1, 1);
    %imshow(frame);
    orangeBin= zeros(720, 1280);
    %sample = [[225 65 50];[240 60 40];[250 170 100]; [235 141 155]; [220 156 157]];
    sample = [[202 78 70]; [235 72 63];[239 38 56];[215 49 32];[196 78 86];
        [260 0 18];[163 34 43];[152 38 38]];
    angle_threshold = 3;
    mag_threshold = 70;
    doubleFrame = double(frame);
    normFrame = sqrt(sum(doubleFrame(:, :, :).^2, 3));

    for i = 1:size(sample, 1)
        temp = sample(i, :);
        %magDiff = abs(normFrame - sqrt(sum((temp).^2)));
        magDiff = abs(doubleFrame(:, :, 1) - temp(1)) + abs(doubleFrame(:, :, 2) - temp(2)) + abs(doubleFrame(:, :, 3) - temp(3));
        innerProduct = temp(1)*doubleFrame(:,:,1) + temp(2)*doubleFrame(:,:,2) + temp(3)*doubleFrame(:,:,3);
        angle = abs(acosd(innerProduct./(norm(temp)*normFrame)));
        orangeBin((angle<angle_threshold) .* (magDiff < mag_threshold) == 1)=250;
    end

    sedisk = strel('disk', 4);
    %orangeBin = imopen(orangeBin, sedisk);
   % subplot(2, 1, 2);
   % imshow(orangeBin);
end