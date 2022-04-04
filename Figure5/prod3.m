function s=prod3(x)
[lx,ly,lz] = size(x);
s = 1;
for i=1:ly
    for j=1:lz
        s = s * x(lx,i,j);
    end
end
