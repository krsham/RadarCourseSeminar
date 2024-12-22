function bSize =bufferSize(steps,iterations_per_save)
    if steps> iterations_per_save
        bSize = iterations_per_save;
    else
        bSize = steps;
    end
end