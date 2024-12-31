function feedSave(fileNameTemplate,currentStep,totalSteps,buffer,buffIndex,buffSize,bufferName)
 
    if buffIndex == buffSize|| currentStep == totalSteps
        outFile = simNameGen(fileNameTemplate);
        % setBuffer(inputname(4),buffer);
        cmd = sprintf('%s = buffer;',bufferName);
        eval(cmd);
        cmd = sprintf('save(outFile,''%s'');',bufferName);
        eval(cmd);
    end
end
% function setBuffer(varName,buffer)
%     assignin('caller',varName,buffer);
% end