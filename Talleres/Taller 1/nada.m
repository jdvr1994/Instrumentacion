x = 1:10;                                           % Create Data (Independent Variable)
V1 = randn(1, 10);                                  % Create Data (Row Vectors)
V2 = randn(1, 10);
V12_sd = std([V1; V2]);                             % Standard Deviation
V12_mean = mean([V1; V2]);                          % Mean
figure(1)
errorbar(x, V12_mean, V12_sd)
grid
