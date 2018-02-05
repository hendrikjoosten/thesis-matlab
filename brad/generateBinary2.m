function binOut = generateBinary2(frame)
    binOut = zeros(720, 1280);
    sticker = [240 10 130];
    sticker2 = [204 63 100];
    band = [70 40 70];
    band2 = [140 80 130];
    %frame = im2double(frame);
    angle_threshold = 5;
    sticker_threshold = 5;
    pixel = double(frame);
    normPixel = sqrt(sum(pixel(:, :, :).^2, 3));
    stickDot = sticker(1)*pixel(:,:,1) + sticker(2)*pixel(:,:,2) + sticker(3)*pixel(:,:,3);
    stickDot2 = sticker2(1)*pixel(:,:,1) + sticker2(2)*pixel(:,:,2) + sticker2(3)*pixel(:,:,3);
    bandDot = band(1)*pixel(:, :, 1) + band(2)*pixel(:,:,2) + band(3)*pixel(:,:,3);
    band2Dot = band2(1)*pixel(:,:,1) + band2(2)*pixel(:,:,2) + band2(3)*pixel(:,:,3);
    stickAngle = acosd(stickDot./(norm(sticker)*normPixel));
    stickAngle2 = acosd(stickDot2./(norm(sticker)*normPixel));
    bandAngle = acosd(bandDot./(norm(band)*normPixel));
    band2Angle = acosd(band2Dot./(norm(band2)*normPixel));
    
    binOut(stickAngle<sticker_threshold)=250;
    binOut(stickAngle2<sticker_threshold)=250;
    binOut(bandAngle<angle_threshold)=250;
    binOut(band2Angle<angle_threshold)=250;
    
%     for x = 1:720
%         for y = 1:1280
%             if(stickAngle(x, y) < sticker_threshold)
%                 binOut(x, y) = 250;
%             elseif (bandAngle(x, y) < angle_threshold)
%                 binOut(x, y) = 250;
%             elseif (band2Angle(x, y) < angle_threshold)
%                 binOut(x, y) = 250;
%             else
%                 binOut(x, y) = 0;
%             end
%                 
%             pixel = double([frame(x, y, 1) frame(x, y, 2) frame(x, y, 3)]);
%             band1Dif = pixel - band;
%             band2Dif = pixel - band2;
%             if(pixel(1) > 180 && pixel(2) < 80)
%                 %temp = double(pixel);
%                  ang = acosd(dot(sticker/norm(sticker), pixel/norm(pixel)));
%                 if(ang<sticker_threshold)
%                     binOut(x, y) = 250;
%                 else
%                     binOut(x, y) = 0;
%                 end
%                 
%             elseif (abs(band1Dif(1)) < 20 && abs(band1Dif(2)) < 20 && abs(band1Dif(3)) < 20)
%                % temp = double(pixel);
%                 ang = acosd(dot(band/norm(band), pixel/norm(pixel)));
%                 if(ang<angle_threshold)
%                     binOut(x, y) = 250;
%                 else
%                     binOut(x, y) = 0;
%                 end
%                 
%             elseif (abs(band2Dif(1)) < 20 && abs(band2Dif(2)) < 20 && abs(band2Dif(3)) < 20)
%                % temp = double(pixel);
%                 ang = acosd(dot(band/norm(band), pixel/norm(pixel)));
%                 if(ang<angle_threshold)
%                     binOut(x, y) = 250;
%                 else
%                     binOut(x, y) = 0;
%                 end
% %             elseif(abs(pixel(1)-band2(1)) < 10 && abs(pixel(2)-band2(2)) < 20 && abs(pixel(3) - band2(3)) < 20)
% %                  temp = double(pixel);
% %                  ang = acosd(dot(band/norm(band), temp/norm(temp)));
% %                 if(ang<angle_threshold)
% %                     binOut(x, y) = 250;
% %                 else
% %                     binOut(x, y) = 0;
% %                 end
%             else
%                 binOut(x, y) = 0;
%             end
%         end
%     end
    sedisk = strel('disk', 4);
    binOut = imopen(binOut, sedisk);
    return
end