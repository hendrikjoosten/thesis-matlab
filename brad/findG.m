function greenBin = findG(frame)
    %figure
    %subplot(2, 1, 1);
    %imshow(frame);
    greenBin= zeros(720, 1280);
    %sample = [[142 154 80]; [182 215 52]; [137 149 77]; [173 175 108]];
    %sample = [sample; [162 166 85]; [179 182 106]; [77 91 47]; [68 72 43]; [76 89 55]];
    sample = [[85 124 66];[157 169 89];[143 141 85];[192 205 124];
        [170 183 72];[162 176 46];[137 152 31];[132 146 48];[180 184 73];
        [143 141 55];[144 146 66];[157 157 75];[162 163 83];[107 110 32];
        [137 149 79];[119 139 44]];
    angle_threshold = 2;
    mag_threshold = 60;
    doubleFrame = double(frame);
    normFrame = sqrt(sum(doubleFrame(:, :, :).^2, 3));

    for i = 1:size(sample, 1)
        temp = sample(i, :);
        magDiff = abs(doubleFrame(:, :, 1) - temp(1)) + abs(doubleFrame(:, :, 2) - temp(2)) + abs(doubleFrame(:, :, 3) - temp(3));
        innerProduct = temp(1)*doubleFrame(:,:,1) + temp(2)*doubleFrame(:,:,2) + temp(3)*doubleFrame(:,:,3);
        angle = abs(acosd(innerProduct./(norm(temp)*normFrame)));
        greenBin((angle<angle_threshold) .* (magDiff < mag_threshold) == 1)=250;
    end

    sedisk = strel('disk', 4);
    %greenBin = imopen(greenBin, sedisk);
    %subplot(2, 1, 2);
    %imshow(greenBin);
end