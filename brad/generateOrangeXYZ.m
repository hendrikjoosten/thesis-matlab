function [orange1, orange2, orangeFound1, orangeFound2] = generateOrangeXYZ(image1, image2)
    orange1 = [0 0];
    orange2 = [0 0];
    orangeFound1 = 1;
    orangeFound2 = 1;
    
    
    orangeBin1 = findO(image1); %generate binary image for orange sticker
    [labeledImage, numRegions] = bwlabeln(orangeBin1, 8); %find # of connected regions
    if(numRegions == 1) %if only 1 region we just record that centroid
        stats = regionprops(labeledImage, 'Centroid');
        centroids = [stats.Centroid];
        centroids = round(centroids);
        centroidx = centroids(1:2:end);
        centroidy = centroids(2:2:end);
        orange1(1) = (centroidx(1));
        orange1(2) = (centroidy(1));
    elseif (numRegions == 0) %if no connected region we move on and set bool false
        orangeFound1 = 0;
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
        orange1(1) = (centroidx(biggestBlobNum));
        orange1(2) = (centroidy(biggestBlobNum));
        
    end

    orangeBin2 = findO(image2); %generate orange bin
    [labeledImage, numRegions] = bwlabeln(orangeBin2, 8); 
    if(numRegions == 1)
        stats = regionprops(labeledImage, 'Centroid');
        centroids = [stats.Centroid];
        centroids = round(centroids);
        centroidx = centroids(1:2:end);
        centroidy = centroids(2:2:end);
        orange2(1) = (centroidx(1));
        orange2(2) = (centroidy(1));
        %generate a z coordinate (all relative to camera 1 (right))
        %orangeXYZ = triangulate(orange1, orange2, stereoParams);

    elseif(numRegions == 0)
        orangeFound2 = 0;
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
        orange2(1) = (centroidx(biggestBlobNum));
        orange2(2) = (centroidy(biggestBlobNum));
        %orangeXYZ = triangulate(orange1, orange2, stereoParams);
    end

    return
end