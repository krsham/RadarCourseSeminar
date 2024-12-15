function w = cancellerCoeffGen(N)
    weightEq = @(i,n) (-1).^(i-1) * (factorial(n))./(factorial(n-i+1).*factorial(i-1));
    weights = weightEq(1:N+1,N);
    w = weights./sum(abs(weights)); %% Normalized
end