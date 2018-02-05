function newEntry = GPS_data(input)
    count = size(input, 1);
    newEntry = zeros(count, 1);
    oldData = input(1,:);
    for i = 2:count
        if(~isequal(input(i, :),oldData))
            newEntry(i) = 1;
        end
        oldData = input(i, :);
    end
end