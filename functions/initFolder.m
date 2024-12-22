 function initFolder(saveFolder,flushFolder) 
    if ~exist(saveFolder,'dir')
        mkdir(saveFolder);
    end
    if flushFolder
        delete([saveFolder,'/*'])
    end
end