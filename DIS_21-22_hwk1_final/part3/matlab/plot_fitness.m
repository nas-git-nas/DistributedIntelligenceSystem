%% Part A: Plot the fitness of Reynolds flock 
[~, ~, ~, data] = read_log("flocking.csv");

figure
hold on;
plot(data.time,data.o)
plot(data.time,data.d)
plot(data.time,data.v)
plot(data.time,data.o.*data.d.*data.v,'LineWidth',2)

legend({'orientation','distance','velocity','combined'})
xlabel('time [s]')
ylabel('fitness')
title('Reynolds flock fitness')

%% Part B: Plot the fitness of Laplacian formation
[~, ~, ~, data] = read_log("formation.csv");

figure
hold on;
plot(data.time,data.d)
plot(data.time,data.v)
plot(data.time,data.d.*data.v,'LineWidth',2)

legend({'distance','velocity','combined'})
xlabel('time [s]')
ylabel('fitness')
title('Laplacian formation fitness')

