function X = generateX(name, start, stop)%name2, start, stop, offset, stereoParams)
    X = [];
    X2 = [];
    obj = VideoReader(name);
    %obj2 = VideoReader(name2);
    for i = start:stop
        figure
        subplot(2, 1, 1);
        temp = read(obj, i);
        %temp2 = read(obj2, i + offset);
               
        
        
        binary = generateBinary2(temp);
        %binary2 = generateBinary2(temp2);
        imshow(temp);
        %figure;
        %imshow(binary2);
        %figure;
        %imshow(temp2);
        [labeledImage, numRegions] = bwlabeln(binary, 8);
        %[labeledImage2, numRegions2] = bwlabeln(binary2, 8);
        stats = regionprops(labeledImage, 'Centroid');
        %stats2 = regionprops(labeledImage2, 'Centroid');
        centroids = [stats.Centroid];
        centroids = round(centroids);
        centroidx = centroids(1:2:end);
        centroidy = centroids(2:2:end);
        
%         centroids2 = [stats2.Centroid];
%         centroids2 = round(centroids2);
%         centroidx2 = centroids2(1:2:end);
%         centroidy2 = centroids2(2:2:end);
        subplot(2, 1, 2);
        imshow(binary);
        %hold on;
        for k = 1:numRegions
            temp = [];
            temp(1) = (centroidx(k)-640);
            temp(2) = -(centroidy(k)-360);
            X = [X;temp];
           % plot(centroidx(k), centroidy(k), 'b*');
        end
%         for k = 1:numRegions2
%             temp2 = [];
%             temp2(1) = (centroidx2(k)-640);
%             temp2(2) = -(centroidy2(k)-360);
%             X2 = [X2;temp2];
%             %plot(centroidx(k), centroidy(k), 'b*');
%         end
        X
   %     X2
        X = sortrows(X, 2)
    %    X2 = sortrows(X, 1)
        %hold off
    end
    return 
end