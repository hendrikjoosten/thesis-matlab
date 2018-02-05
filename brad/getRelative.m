function [relXY1, relXY2] = getRelative(P1, P2, P3, bools)
    
for i = 2: size(P1, 1)
   if(~bools(i))
    P1(i) = P1(i-1);
   end
end

relXY1 = P2 - P1;
relXY2 = P3 - P1;
    
end