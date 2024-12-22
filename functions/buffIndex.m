function bIndex = buffIndex(currentIndex,maxSize)
    rem = mod(currentIndex,maxSize);
    bIndex(rem==0) = maxSize;
    bIndex(rem~=0) = rem(rem~=0);
end