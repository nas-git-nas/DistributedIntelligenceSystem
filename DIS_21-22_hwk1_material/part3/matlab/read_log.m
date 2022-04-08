function [N_SIM, T_SIM, T, data] = read_log(f)
%% Read log file and create simulation variables

path = "../webots/controllers/flocking_super/"+f;

temp = importdata(path);

data = [];
ind = 0;
for f_ = str2mat(temp.colheaders)'
    ind = ind + 1;
    data.(strrep(f_',' ','')) = temp.data(:,ind);
end

% remove trailing NaN
fn = fieldnames(data);
for k=1:numel(fn)
    data.(fn{k}) = data.(fn{k})(1:end-1); 
end

N_SIM = length(data.time);
T_SIM = 1: N_SIM;
T = data.time(2) - data.time(1);

end