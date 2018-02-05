function [pink1, pink2, pinkFound1, pinkFound2] = generatePinkXYZ(image1, image2)
pink1 = [0 0];
pink2 = [0 0];
pinkFound1 = 1;
pinkFound2 = 1;


pinkBin1 = findP(image1); %generate binary image for pink band
[labeledImage, numRegions] = bwlabeln(pinkBin1, 8); %find # of connected regions
if(numRegions == 1) %if only 1 region we just record that centroid
    stats = regionprops(labeledImage, 'Centroid');
    centroids = [stats.Centroid];
    centroids = round(centroids);
    centroidx = centroids(1:2:end);
    centroidy = centroids(2:2:end);
    pink1(1) = (centroidx(1));
    pink1(2) = (centroidy(1));
elseif (numRegions == 0) %if no connected region we move on and set bool false
    pinkFound1 = 0;
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
    pink1(1) = (centroidx(biggestBlobNum));
    pink1(2) = (centroidy(biggestBlobNum));
    
end

pinkBin2 = findP(image2); %generate pink bin
[labeledImage, numRegions] = bwlabeln(pinkBin2, 8);
if(numRegions == 1)
    stats = regionprops(labeledImage, 'Centroid');
    centroids = [stats.Centroid];
    centroids = round(centroids);
    centroidx = centroids(1:2:end);
    centroidy = centroids(2:2:end);
    pink2(1) = (centroidx(1));
    pink2(2) = (centroidy(1));
    %generate a z coordinate (all relative to camera 1 (right))
    %pinkXYZ = my_triang(pink1, pink2);
    %pinkXYZ = triangulate(pink1, pink2, stereoParams);
    
elseif(numRegions == 0)
    pinkFound2 = 0;
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
    pink2(1) = (centroidx(biggestBlobNum));
    pink2(2) = (centroidy(biggestBlobNum));
    %pinkXYZ = my_triang(pink1, pink2);
    %pinkXYZ = triangulate(pink1, pink2, stereoParams);
end

return
end