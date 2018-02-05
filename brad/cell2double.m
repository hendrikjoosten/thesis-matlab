function converted = cell2double(vec)
converted = [];
for i = 1: size(vec)
    a = cell2mat(vec(i));
    converted = [converted; str2double(a)];
end
end