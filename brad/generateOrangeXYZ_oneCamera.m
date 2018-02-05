function [orange1, orangeFound] = generateOrangeXYZ_oneCamera(image1)
    orange1 = [0 0];
    orangeFound = 1;
    
    orangeBin1 = findO(image1); %generate binary image for orange sticker
    [labeledImage, numRegions] = bwlabeln(orangeBin1, 8); %find # of connected regions
    if(numRegions == 1) %if only 1 region we just record that centroid
        stats = regionprops(labeledImage, 'Centroid');
        centroids = [stats.Centroid];
        centroids = round(centroids);
        centroidx = centroids(1:2:end);
        centroidy = centroids(2:2:end);
        orange1(1) = (centroidx(1)-640)/8.5;
        orange1(2) = (centroidy(1)-360)/8.5;
    elseif (numRegions == 0) %if no connected region we move on and set bool false
        orangeFound = 0;
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
        orange1(1) = ((centroidx(biggestBlobNum)-640)/8.5);
        orange1(2) = (centroidy(biggestBlobNum)-360)/8.5;
        
    end
    return
end