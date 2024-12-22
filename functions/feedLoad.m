function bufferOut = feedLoad(fileSourceFolder,step,bufferStep,bufferSize,bufferName,bufferIn)
        sourceFiles = what(fileSourceFolder);
        sourceFiles = sourceFiles.mat;
        if bufferStep == 1 || step == 1
            fileIndex = floor(step/bufferSize) + 1;
            inFile = [fileSourceFolder,'/',sourceFiles{fileIndex}];
            bufferStruct = load(inFile,bufferName);
            bufferOut = struct2array(bufferStruct);
        else
            bufferOut = bufferIn;
        end
end

