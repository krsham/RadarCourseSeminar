function simName = simNameGen(template)
    postFix = char(datetime);
    simName = [template,'_',postFix];
end