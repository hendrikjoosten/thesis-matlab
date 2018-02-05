function [x, y, found] = findLargest(frame)
    found = 1;
    x = 0;
    y = 0;
    [labeledImage, numRegions] = bwlabeln(frame, 8); %find # of connected regions
    if(numRegions == 1) %if only 1 region we just record that centroid
        stats = regionprops(labeledImage, 'Centroid');
        centroids = [stats.Centroid];
        centroids = round(centroids);
        centroidx = centroids(1:2:end);
        centroidy = centroids(2:2:end);
        x = (centroidx(1));
        y = (centroidy(1));
    elseif (numRegions == 0) %if no connected region we move on and set bool false
        found = 0;
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
        x = (centroidx(biggestBlobNum));
        y = (centroidy(biggestBlobNum));
        
    end
end