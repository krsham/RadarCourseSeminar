function bufferOut = feedLoad(varargin)
    fileSourceFolder = varargin{1};
    if nargin > 3
        step = varargin{2};
        bufferStep = varargin{3};
        bufferSize = varargin{4};
        bufferName = varargin{5};
        bufferIn = varargin{6};
        sourceFiles = what(fileSourceFolder);
        sourceFiles = sourceFiles.mat;
        if bufferStep == 1 || step == 1 
            fileIndex = floor(step/bufferSize) + 1;
            inFile = [fileSourceFolder,'/',sourceFiles{fileIndex}];
            % tic
            bufferStruct = load(inFile,bufferName);
            % toc
            bufferOut = struct2array(bufferStruct);
        else
            bufferOut = bufferIn;
        end
    else
        fileIndex = varargin{2};
        sourceFiles = what(fileSourceFolder);
        sourceFiles = sourceFiles.mat;
        inFile = [fileSourceFolder,'/',sourceFiles{fileIndex}];
        buff = load(inFile);
        bufferOut = buff;
    end
end

