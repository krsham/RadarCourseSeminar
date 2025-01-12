function [folderName,templateDef] = saveFolderDef(rootDir,destinationDir,fileName)
    folderName                = [rootDir,'/',destinationDir];
    templateDef              = [folderName,'/',fileName];
end