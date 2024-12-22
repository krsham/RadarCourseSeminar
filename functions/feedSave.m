function feedSave(fileNameTemplate,currentStep,totalSteps,buffer,buffIndex,buffSize)
 
    if buffIndex == buffSize|| currentStep == totalSteps
        outFile = simNameGen(fileNameTemplate);
        setBuffer(inputname(4),buffer); 
        save(outFile,inputname(4));
    end
end
function setBuffer(varName,buffer)
    assignin('caller',varName,buffer);
end