function selectRange = batchSelection(batchSize,startIndex)
selectRange = (startIndex-1)*batchSize+1:startIndex*batchSize;
end