function simName = simNameGen(template)
    postFix = [char(datetime),char(randi([48,57]))]; %% add a random number to prevent overwrite
    simName = [template,'_',postFix];
end