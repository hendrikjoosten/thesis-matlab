function binOut = generateBinary(frame)
    binOut = zeros(720, 1280);
    sticker = [240 10 130];
    band = [70 40 70];
    band2 = [140 80 130];
    %frame = im2double(frame);
    angle_threshold = 6;
    sticker_threshold = 15;
    for x = 1:720
        for y = 1:1280
            pixel = double([frame(x, y, 1) frame(x, y, 2) frame(x, y, 3)]);
            band1Dif = pixel - band;
            band2Dif = pixel - band2;
            if(pixel(1) > 180 && pixel(2) < 80)
                %temp = double(pixel);
                 ang = acosd(dot(sticker/norm(sticker), pixel/norm(pixel)));
                if(ang<sticker_threshold)
                    binOut(x, y) = 250;
                else
                    binOut(x, y) = 0;
                end
                
            elseif (abs(band1Dif(1)) < 20 && abs(band1Dif(2)) < 20 && abs(band1Dif(3)) < 20)
               % temp = double(pixel);
                ang = acosd(dot(band/norm(band), pixel/norm(pixel)));
                if(ang<angle_threshold)
                    binOut(x, y) = 250;
                else
                    binOut(x, y) = 0;
                end
                
            elseif (abs(band2Dif(1)) < 20 && abs(band2Dif(2)) < 20 && abs(band2Dif(3)) < 20)
               % temp = double(pixel);
                ang = acosd(dot(band/norm(band), pixel/norm(pixel)));
                if(ang<angle_threshold)
                    binOut(x, y) = 250;
                else
                    binOut(x, y) = 0;
                end
%             elseif(abs(pixel(1)-band2(1)) < 10 && abs(pixel(2)-band2(2)) < 20 && abs(pixel(3) - band2(3)) < 20)
%                  temp = double(pixel);
%                  ang = acosd(dot(band/norm(band), temp/norm(temp)));
%                 if(ang<angle_threshold)
%                     binOut(x, y) = 250;
%                 else
%                     binOut(x, y) = 0;
%                 end
            else
                binOut(x, y) = 0;
            end
        end
    end
    sedisk = strel('disk', 4);
    binOut = imopen(binOut, sedisk);
    return
end