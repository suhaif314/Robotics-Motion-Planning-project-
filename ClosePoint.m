function [smallestarray] = ClosePoint(map,q)
x = size(map,1);
Dmin = 10;
smallestarray = 1;
for i = 1:x
    d = norm(map(i, 1:end-1)-q);

    %check for the collision

    if d< Dmin
        smallestarray = i;
        Dmin = d;
    end
end
end
