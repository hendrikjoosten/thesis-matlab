function pinkBin = findP(frame)
     %subplot(2, 1, 1);
     %imshow(frame);
    pinkBin= zeros(720, 1280);
    %sample = [[111 49 70];[100 48 61];[121 68 86];[76 37 37];[150 101 103]];
    %sample = [sample ; [175 82 100];[98 65 74];[97 50 70];[149 82 83]; [85 49 36]];
    %sample = [sample; [167 84 76]; [72 30 24]; [65 35 25]; [87 47 38]];
    sample = [[181 89 152];[250 149 197];[103 59 77];[173 114 113];[219 140 162];
        [174 108 123];[202 132 158];[76 37 37];[188 89 113];[133 93 102];
        [141 84 87];
        [105 51 74];[163 102 98];[123 66 62];[112 70 91];
        [104 51 69];[114 81 99];[90 52 60];[125 41 51];[150 76 85];[140 57 67];
        [166 83 75];[144 67 61];[177 93 106];[109 44 80];[166 77 120];
        [166 88 123]];
    angle_threshold = 3;
    mag_threshold = 80;
    doubleFrame = double(frame);
    normFrame = sqrt(sum(doubleFrame(:, :, :).^2, 3));

    for i = 1:size(sample, 1)
        temp = sample(i, :);
        %magDiff = abs(normFrame - sqrt(sum((temp).^2)));
        magDiff = abs(doubleFrame(:, :, 1) - temp(1)) + abs(doubleFrame(:, :, 2) - temp(2)) + abs(doubleFrame(:, :, 3) - temp(3));
        innerProduct = temp(1)*doubleFrame(:,:,1) + temp(2)*doubleFrame(:,:,2) + temp(3)*doubleFrame(:,:,3);
        angle = abs(acosd(innerProduct./(norm(temp)*normFrame)));
        pinkBin((angle<angle_threshold) .* (magDiff < mag_threshold) == 1)=250;
    end

    sedisk = strel('disk', 4);
    pinkBin = imopen(pinkBin, sedisk);
    %subplot(2, 1, 2);
    %figure
    imshow(pinkBin);
end