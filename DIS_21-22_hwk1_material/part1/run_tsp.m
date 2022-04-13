function run_tsp(runs,experiment,cities,localsearch)
if(nargin<4) localsearch=0; end
fit_vector = zeros(1,runs);
fit_2opt_vector = zeros(1,runs);
for i = 1:runs
    fprintf('\n Run number %d\n', i);
    [~,~,fit_vector(i), fit_2opt_vector(i)]=tsp(experiment,cities,localsearch);
end
%% <<<<<<-------------------- YOUR CODE HERE ---------------->>>>>>
% Implement mean and standard deviation of the runs
fit_mean = mean(fit_vector)
fit_std = std(fit_vector)

%% <<<<<<---------------------------------------------------->>>>>>

if(localsearch)
    %% <<<<<<-------------------- YOUR CODE HERE ---------------->>>>>>
    % Implement mean and standard deviation of the runs if localsearch is equal to 1
    fit_2opt_mean = mean(fit_2opt_vector)
    fit_2opt_std = std(fit_2opt_vector)

    %% <<<<<<---------------------------------------------------->>>>>>

end