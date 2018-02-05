function [green1, green2, greenFound1, greenFound2] = generateGreenXYZ(image1, image2)
    green1 = [0 0];
    green2 = [0 0];
    greenFound1 = 1;
    greenFound2 = 1;
    
    greenBin1 = findG(image1); %generate binary image for green band
    [labeledImage, numRegions] = bwlabeln(greenBin1, 8); %find # of connected regions
    if(numRegions == 1) %if only 1 region we just record that centroid
        stats = regionprops(labeledImage, 'Centroid');
        centroids = [stats.Centroid];
        centroids = round(centroids);
        centroidx = centroids(1:2:end);
        centroidy = centroids(2:2:end);
        green1(1) = (centroidx(1));
        green1(2) = (centroidy(1));
    elseif (numRegions == 0) %if no connected region we move on and set bool false
        greenFound1 = 0;
    else %otherwise we need to find largest connected region
        %extract centroid and area of all connected regions
        stats = regionprops(labeledImage, 'Centroid', 'area');
        allAreas = [stats.Area];
        biggestArea = 0;
        biggestBlobNum = 1;
        %find index of largest area region
        for k = 1:numRegions
            if(allAreas(k) > biggestArea)
                biggestBlobNum = k;
                biggestArea = allAreas(k);
            end
        end
        %record centroid of blob at largest area index
        centroids = [stats.Centroid];
        centroids = round(centroids);
        centroidx = centroids(1:2:end);
        centroidy = centroids(2:2:end);
        green1(1) = (centroidx(biggestBlobNum));
        green1(2) = (centroidy(biggestBlobNum));
        
        
    end

    greenBin2 = findG(image2); %generate green bin
    [labeledImage, numRegions] = bwlabeln(greenBin2, 8); 
    if(numRegions == 1)
        stats = regionprops(labeledImage, 'Centroid');
        centroids = [stats.Centroid];
        centroids = round(centroids);
        centroidx = centroids(1:2:end);
        centroidy = centroids(2:2:end);
        green2(1) = (centroidx(1));
        green2(2) = (centroidy(1));
        %generate a z coordinate (all relative to camera 1 (right))
        %greenXYZ = my_triang(green1, green2);
        %greenXYZ = triangulate(green1, green2, stereoParams);

    elseif(numRegions == 0)
        greenFound2 = 0;
    else
        %Find largest connected region
        stats = regionprops(labeledImage, 'Centroid', 'area');
        allAreas = [stats.Area];
        biggestArea = 0;
        biggestBlobNum = 1;
        for k = 1:numRegions
            if(allAreas(k) > biggestArea)
                biggestBlobNum = k;
                biggestArea = allAreas(k);
            end
        end
        centroids = [stats.Centroid];
        centroids = round(centroids);
        centroidx = centroids(1:2:end);
        centroidy = centroids(2:2:end);
        green2(1) = (centroidx(biggestBlobNum));
        green2(2) = (centroidy(biggestBlobNum));
        %greenXYZ = my_triang(green1, green2);
        %greenXYZ = triangulate(green1, green2, stereoParams);
    end
   % greenXYZ(1) = greenXYZ(1) - 640;
   % greenXYZ(2) = -(greenXYZ(1) + 360);
    return
end