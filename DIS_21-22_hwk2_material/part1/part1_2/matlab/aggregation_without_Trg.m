% Usual cleaning
clc
close all
clear all

% Debugging
debugging = 0;
verbose = 1;

% Initialize parameters
Nr = 5; % Total number of robots
Ns = 10; % Total number of seeds
Wsf0 = Nr; % Number of robots in search-free state
Wsl0 = 0; % Number of robots in search-loaded state
Wr0 = 0; % Number of robots in releasing state
Wg0 = 0; % Number of robots in gripping state
Wof0 = 0; % Number of robots in obstacle avoidance-free state
Wol0 = 0; % Number of robots in obstacle avoidance-floaded state
k_sim = 1000; %10080; % Total simulation time
vr = 0.08; %average robot speed, m/s
rs = 0.1; %detection radius of the distance sensor
ds = 2*rs; %detection distance of the sensor
da = 1; % edge of the arena
Aa = da^2; % Area of the arena, m^2
Ts = 1; % Sample time, s
Trg = 2; % Releasing and gripping delay, s
Toa = 3; % Obstacle avoidance delay, s
alpha_inc = 60; % Construction angle
alpha_dec = 60; % Destruction angle
Td_max = max(Trg,Toa)+1; % Simulation start
p_thr_sbc = 0.1; % Probability threshold for size of the biggest cluster

% Allocate variables
Wsf = zeros(k_sim,1);
Wsl = zeros(k_sim,1);
Wr = zeros(k_sim,1);
Wg = zeros(k_sim,1);
Wof = zeros(k_sim,1);
Wol = zeros(k_sim,1);
Ncn = zeros(k_sim,Ns,1); % Number of clusters of size n (max Ns)
NC = zeros(k_sim,1); % Number of clusters NC
ACS = zeros(k_sim,1); % Average cluster size ACS
SBC = zeros(k_sim,1);

p1 = zeros(k_sim,1);
p2 = zeros(k_sim,1);
nb_seeds_loaded = zeros(k_sim,1);
nb_seeds_in_cluster = zeros(k_sim,1);
inc_rate = zeros(k_sim,1);
dec_rate = zeros(k_sim,1);

% Initialize variables
Wsf(Td_max) = Wsf0;
Wsl(Td_max) = Wsl0;
Wr(Td_max) = Wr0;
Wg(Td_max) = Wg0;
Wol(Td_max) = Wol0;
Wof(Td_max) = Wof0;
Ncn(1:Td_max,1) = Ns;
NC(Td_max) = 0;
ACS(Td_max) = 0;
SBC(Td_max) = 0;

p_wall = (da^2 - (da-ds)^2)/Aa; %vr/(da-2*ds)
p_robot = (Nr-1)*(pi*rs^2)/Aa;

p3 = p_wall + p_robot;
p4 = p_wall + p_robot;

penc = zeros(Ns,1);
pdec = zeros(Ns,1);
pinc = zeros(Ns,1);
for n = 1:Ns % iteration: [1,Ns] <-> cluster size: [1,Ns]        
%     penc(n) = n*(pi*rs^2)/Aa;
    penc(n) = n*(Ts*vr*ds)/Aa;
    if n == 1
        pdec(n) = penc(n);
        pinc(n) = penc(n);
    else
        pdec(n) = (alpha_dec/180) * penc(n);       
        pinc(n) = (alpha_inc/180) * penc(n);
    end
end
if(verbose)
    disp(["pdec= ", pdec(:)']);
    disp(["pinc= ", pinc(:)']);
end

% Perform the simulation
for k = Td_max:k_sim

    for n = 1:Ns % iteration: [1,Ns] <-> cluster size: [1,Ns]           
        if n == 1 % loaded robot does not drop seed in free space
            Ncn(k+1,n) = Ncn(k,n) - pdec(n)*Ncn(k-Trg,n)*Wsf(k-Trg) + pdec(n+1)*Ncn(k-Trg,n+1)*Wsf(k-Trg) ...
                                  - pinc(n)*Ncn(k,n)*Wsl(k);

            inc_rate(k) = inc_rate(k) + pinc(n)*Ncn(k,n)*Wsl(k);
            dec_rate(k) = dec_rate(k) + pdec(n)*Ncn(k,n)*Wsf(k) + pdec(n+1)*Ncn(k-Trg,n+1)*Wsf(k-Trg);
        elseif n == Ns % no larger clusters than of size Ns exist
            Ncn(k+1,n) = Ncn(k,n) - pdec(n)*Ncn(k-Trg,n)*Wsf(k-Trg) + pinc(n-1)*Ncn(k-Trg,n-1)*Wsl(k-Trg);

            inc_rate(k) = inc_rate(k) + pinc(n-1)*Ncn(k-Trg,n-1)*Wsl(k-Trg);
            dec_rate(k) = dec_rate(k) + pdec(n)*Ncn(k,n)*Wsf(k);
        else
            Ncn(k+1,n) = Ncn(k,n) - pdec(n)*Ncn(k-Trg,n)*Wsf(k-Trg) + pdec(n+1)*Ncn(k-Trg,n+1)*Wsf(k-Trg) ...
                                  - pinc(n)*Ncn(k,n)*Wsl(k) + pinc(n-1)*Ncn(k-Trg,n-1)*Wsl(k-Trg);

            inc_rate(k) = inc_rate(k) + pinc(n)*Ncn(k,n)*Wsl(k) + pinc(n-1)*Ncn(k-Trg,n-1)*Wsl(k-Trg);
            dec_rate(k) = dec_rate(k) + pdec(n)*Ncn(k,n)*Wsf(k) + pdec(n+1)*Ncn(k-Trg,n+1)*Wsf(k-Trg);

        end                          

        if debugging
            % ERROR verification: Ncn range
            if Ncn(k+1,n) < 0
                disp("ERROR: Ncn < 0");
                disp(["iteration =", k, "nb. seeds", n, "Ncn=", Ncn(k+1,n)]);
                break;
            elseif Ncn(k+1,n) > Ns
                disp("ERROR: Ncn > Ns");
                disp(["iteration =", k, "nb. seeds", n, "Ncn=", Ncn(k+1,n)]);
                break;
            end
        end

        p1(k) = p1(k) + pdec(n)*Ncn(k,n);
        p2(k) = p2(k) + pinc(n)*Ncn(k,n);
    end

    if debugging
        % ERROR verification: p1 and p2 range
        % ERROR verification: p1 and p2 range
        if p1(k)<0 || p1(k)>1
            disp("ERROR: p1 out of range");
            disp(["iteration =", k, "p1=", p1(k)]);
            disp(["pdec", round(pdec(:)',2)]);
            disp(["Ncn", Ncn(k,:)]);
            break;
        end
        if p2(k)<0 || p2(k)>1
            disp("ERROR: p2 out of range");
            disp(["iteration =", k, "p2=", p2(k)]);
            disp(["pdec", pinc(:)]);
            disp(["Ncn", Ncn(k,:)]);
            break;
        end
    end
    

    
    Wsf(k+1) = Wsf(k) - p3*(Wsf(k)-Wsf(k-Toa)) - p1(k)*Wsf(k) + p2(k-Trg)*Wsl(k-Trg);
    Wof(k+1) = Wof(k) + p3*(Wsf(k)-Wsf(k-Toa));
    Wg(k+1) = Wg(k) + p1(k)*Wsf(k) - p1(k-Trg)*Wsf(k-Trg);
    Wsl(k+1) = Wsl(k) - p4*(Wsl(k)-Wsl(k-Toa)) - p2(k)*Wsl(k) + p1(k-Trg)*Wsf(k-Trg);
    Wol(k+1) = Wol(k) + p4*(Wsl(k)-Wsl(k-Toa));
    Wr(k+1) = Wr(k) + p2(k)*Wsl(k) - p2(k-Trg)*Wsl(k-Trg);


    
    % ERROR verification: Wtot
    if debugging
        Wtot = Wsf(k)+Wof(k)+Wg(k)+Wsl(k)+Wol(k)+Wr(k);
        if abs(Wtot-Nr) > p_thr_sbc
            disp("ERROR: Wtot not equal to Nr");
            disp(["iteration =", k, "Wtot=", Wtot]);
            break;
        end
    end
    
    % calc. nb. seeds loaded by a robot (incl. gripping and releasing)
    nb_seeds_loaded(k+1) = nb_seeds_loaded(k) + p1(k)*Wsf(k) - p2(k-Trg)*Wsl(k-Trg);

    % ERROR verification: nb. seeds loaded
    if debugging
        if abs(nb_seeds_loaded-Wsl(k)-Wol(k)-Wg(k)-Wr(k)) > p_thr_sbc
            disp("ERROR: nb. seeds loaded not consistent");
            disp(["iteration =", k, "nb_seeds_loaded=", nb_seeds_loaded(k)]);
            disp(["Wsf", Wsf(k), "Wsl", Wsl(k), "Wof", Wof(k), "Wol", Wol(k), "Wg", Wg(k), "Wr", Wr(k)]);
            break;
        end
    end


    % Metric NC: nb. clusters
    NC(k) = sum(Ncn(k,1:Ns));
    
    % Metric SBC: size of largest cluster
    for n = 1:Ns % iteration: [1,Ns] <-> cluster size: [1,Ns]          
        if Ncn(k,n) > p_thr_sbc % choosing threshold because it is probabilistic
            SBC(k) = n;
        end
    end

    % Metric ACS: avg. cluster size
    for n = 1:Ns % calc. nb. seeds inside all clusters
        nb_seeds_in_cluster(k) = nb_seeds_in_cluster(k) + n*Ncn(k,n); 
    end
    ACS(k) = nb_seeds_in_cluster(k)/NC(k);

    
     

    % ERROR verification: conservation of nb. of seeds  
    if debugging 
        if abs(nb_seeds_in_cluster(k)+nb_seeds_loaded(k)-Ns) > p_thr_sbc
            disp("ERROR: nb. seeds not conserved");
            disp(["iteration =", k, "nb_seeds_in_cluster=", nb_seeds_in_cluster(k), ...
                  "nb_seeds_loaded=", nb_seeds_loaded(k)]);
            disp(["n=", n,"Ncn=", Ncn(k,:)]);
            break;
        end
    end
    
    
end


%%
% Plot number of robots in each state
figure
plot(Wsf,'LineWidth',2)
hold on
grid on
plot(Wsl,'LineWidth',2)
hold on
plot(Wg,'LineWidth',2)
hold on
plot(Wr,'LineWidth',2)
hold on
plot(Wof,'LineWidth',2)
hold on
plot(Wol,'LineWidth',2)
hold on
plot(Wsf + Wsl + Wg + Wr + Wof + Wol, 'LineWidth',2);
legend('Wsf','Wsl','Wg','Wr','Wof','Wol','Wtot');
xlabel('Discrete time (k)');
ylabel('Number');

% Plot metrics
figure
plot(NC,'LineWidth',2)
hold on
grid on
plot(ACS,'LineWidth',2)
hold on
plot(SBC,'LineWidth',2)
legend('NC','ACS','SBC');
xlabel('Discrete time (k)');
ylabel('Number');

% Plot nb. clusters of size n
figure
for n = 1:Ns
    plot(Ncn(:,n),'LineWidth',2)
    hold on
    grid on
end
legend('Ncn(1)','Ncn(2)','Ncn(3)','Ncn(4)','Ncn(5)','Ncn(6)','Ncn(7)', ...
        'Ncn(8)','Ncn(9)','Ncn(10)');
xlabel('Discrete time (k)');
ylabel('Number');

% Plot nb seeds inside cluster and loaded
figure
plot(nb_seeds_in_cluster,'LineWidth',2)
hold on
grid on
plot(nb_seeds_loaded(1:k_sim),'LineWidth',2)
hold on
grid on
plot(nb_seeds_in_cluster+nb_seeds_loaded(1:k_sim),'LineWidth',2)
legend('nb seeds cluster', 'nb seeds loaded', 'total nb seeds');
xlabel('Discrete time (k)');
ylabel('Number');


% Plot prob. p1 and p2
figure
plot(p1,'LineWidth',2)
hold on
grid on
plot(p2,'LineWidth',2)
legend('p1', 'p2');
xlabel('Discrete time (k)');
ylabel('Number');

% Plot increase and decrease rate
figure
plot(inc_rate,'LineWidth',2)
hold on
grid on
plot(dec_rate,'LineWidth',2)
legend('inc rate', 'dec rate');
xlabel('Discrete time (k)');
ylabel('Number');



