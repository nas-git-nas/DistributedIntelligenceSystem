function [solution,solution_2opt, fit, fit_2opt]=tsp(experiment,cities,localsearch)
%% function tsp(experiment,cities,localsearch)
%
% The Travelling Salesman Problem is specified by a list of cities (x and y
% coordinates in the columns). 'Cities' is a Nx2 vector, with N the number
% of cities.
% The paramter 'localsearch' allows you to specify, whether you want to
% make use of local search (1), or not (0, default).
%
% Suitable values for 'experiment' are 
% 'ant'    : Calculates a solution for the TSP using Elitist Ant System
%            (EAS) and uses local search for optimization.
%
% (c) Nikolaus Correll, Sven Gowal, Amanda Prorok, Swarm-Intelligent Systems Group, EPFL, 2006.
% modification, Bahar Haghighat, 2015.
% Modified by Chiara Ercolani, 2022.
  
solution = 0;
solution_2opt = 0;
fit = 0;
fit_2opt = 0;

if(nargin<3) localsearch=0; end

distance_matrix =calculate_distance_matrix(cities);
switch (experiment)   
   case 'ant'
        alpha=5;
        beta=5; % initial value 5
        ants=20;
        tours=50; 
        [ant_solution,best]=solvetsp_ant(cities,distance_matrix,ants,tours,alpha,beta);

        figure(3);        
        clf;
        if(localsearch), subplot(1,2,1); end
        plotsolution(cities,ant_solution,sprintf('EAS solution without local search:\n %2.3f',fitness(ant_solution,distance_matrix)));
        fit = fitness(ant_solution,distance_matrix);

        if(localsearch)
        subplot(1,2,2);
        tic, solution_2opt=local_search_2opt(ant_solution,cities,distance_matrix); toc
        fit_2opt = fitness(solution_2opt,distance_matrix);
        plotsolution(cities,solution_2opt,sprintf('EAS solution with 2-opt local search:\n %2.3f',fitness(solution_2opt,distance_matrix)));
        end
        
        solution = ant_solution;
        
    otherwise
        error(sprintf('Unknown option: %s',experiment))
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [distance_matrix]=calculate_distance_matrix(cities)
%% function [distance_matrix]=calculate_distance_matrix(cities);
% Calculates a NxN distance matrix from a 2xN vector specifying city
% coordinates.
distance_matrix=zeros(size(cities,1));
for I=1:size(cities,1)
    for J=1:size(cities,1)
        distance_matrix(I,J)=distance(cities(I,:),cities(J,:));
    end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
function [solution,best]=solvetsp_ant(cities,distance_matrix,nants,ntours,A,B)
%% function [solution,best]=solvetsp_ant(cities,distance_matrix,nants,ntours,alpha,beta)
%
% Solves the travelling salesman problem by implementing a basic version of
% ant system.
% 
% 'nants' : the number of ants
% 'ntours': the number of tours per ant
% 'alpha' : the weight for pheromone information
% 'beta'  : the weight for heuristic information
       
solutions=zeros(nants,length(cities)); % contains the solutions found by each ant during a single tour, ants*cities matrix
solution=[]; % will be returned as the best route found, over all ants and over all tours 
best=intmax; % will contain the fitness of the best route
% 1. Initialize pheromon trail
pheromonetrail=ones(size(cities,1))*0.1; % uniform distribution
% 2. Deploy ants, generate tours, deploy pheromones, and iterate
for K=1:ntours % number of tours is like the number of iterations 
    % a. Deploy ants
    % Ants are deployed to a random city. The matrix
    % 'unvisited_cities' keeps track of the unvisited cities for ant 'I'
    
    alpha=A;
    beta=B;
    
    %%<<<<<<<<--------------- YOUR CODE HERE----------->>>>>>>
    % Make the values of alpha and beta change linearly with time
    % The more time passes, the more the ant should rely on pheromones
    alpha = 2*(K/ntours)*A;
    beta = 2*((ntours-K)/ntours)*B;


    %Pheromone evaporation of 10% at each iteration
    if(K~=1)
        pheromonetrail = pheromonetrail.*0.9;
    end
    
    %%<<<<<<<<---------------------------------------->>>>>>>
    
    unvisited_cities=1:length(cities); % [1 2 3 ... N]
    current_city = 1:nants;
    for I=1:nants
        current_city(I)=unvisited_cities(round(rand()*(length(unvisited_cities)-1))+1); % for each ant, randomly assign a starting city
        solutions(I,:)=current_city(I); % the first city in the solution of each ant is the current city
    end
    unvisited_cities=[]; % clear unvisited cities
    for I=1:nants
        unvisited_cities=[unvisited_cities; delete_entry(1:length(cities),current_city(I))]; % ants*cities matrix, each row i corresponds to the label of cities still unvisited by an ant i
    end
    % b. Generate one tour for each ant
    for J=2:length(cities) % the city to begin with is already determined
        unvisited_cities_next=[];
        for I=1:nants
            % The first and second column of the matrix 'data' contains the heuristic
            % and pheromone information, respectively.
            data=[1./distance_matrix(current_city(I),unvisited_cities(I,:))', pheromonetrail(current_city(I),unvisited_cities(I,:))'];
            
            Q = (data(:,1).^beta).*(data(:,2).^alpha);
            p = Q./sum(Q);                  
            
            % We select a random city with probability p
            [p,sortindex]=sort(p,1,'descend'); % sorts elements in p large:small
            unvisited_cities(I,:)=unvisited_cities(I,sortindex); % unvisited cities are sorted accrding to p, most probable to least probable
            current_city(I)=unvisited_cities(I,min(find(rand()<cumsum(p)))); % find returns the indices of all nonzero elements, small to big; choose the next city randomly
            solutions(I,J)=current_city(I);
            unvisited_cities_next=[unvisited_cities_next; delete_entry(unvisited_cities(I,:),current_city(I))];
        end
        unvisited_cities=unvisited_cities_next; % update unvisited cities for the next step
    end
    
    % c. Calculate pheromones to deploy
    fitnesses=[];
    for I=1:size(solutions,1) % looking over ant's solutions
        fitnesses(I)=fitness(solutions(I,:),distance_matrix); % fitness of a tour is the length of the tour
    end
    
    for N=1:nants
        for I=1:(length(cities)-1)
            pheromonetrail(solutions(N,I),solutions(N,I+1))= pheromonetrail(solutions(N,I),solutions(N,I+1)) + 1/fitnesses(N);
        end
        pheromonetrail(solutions(N,1),solutions(N,length(cities)))= pheromonetrail(solutions(N,1),solutions(N,length(cities))) + 1/fitnesses(N);
    end
    
    % Find elite ant
    elitist=find(fitnesses==min(fitnesses));
    elitist=elitist(1);
    for I=1:(length(cities)-1)
        pheromonetrail(solutions(elitist,I),solutions(elitist,I+1))= pheromonetrail(solutions(elitist,I),solutions(elitist,I+1)) + 1/fitnesses(elitist);
    end
    pheromonetrail(solutions(elitist,1),solutions(elitist,length(cities)))= pheromonetrail(solutions(elitist,I),solutions(elitist,length(cities)))+ 1/fitnesses(elitist);
    

    
    % Store best solution
    if(fitnesses(elitist)<best)
        solution=solutions(elitist,:);
        best=fitnesses(elitist);
        fprintf('Best route (%f) found in tour %d\n',best,K);
    end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function solution=local_search_2opt(solution,cities,distance_matrix)
%% function solution=local_search_2opt(solution,cities,distance_matrix)
%
% Performs a local search by systematically searching ALL permutations of a
% solution until no better solution is found.
run=1;
best_fitness_yet=fitness(solution,distance_matrix);
while(run)
    run=0;
    for I=1:length(solution)
        for J=I:length(solution)-1
            if(I~=J)
                %%<<<<<<<---------YOUR CODE HERE------------>>>>>>
                % opt_solution should contain one possible permutation of
                % 'solution' that can be obtained by swapping two edges.
                % Hint: you can swap the order of a vector as follows:
                % Given A=[4 5 6], you can swap the order of its content by accessing 
                % it like A([3 2 1]) -> A=[6 5 4]
                
                %<---- 
                % To do: Your code here: permute the edges here to implement a 2-opt local
                % search.
                previous_solution = solution;

                swap_indexes = 1:length(solution);
                swap_indexes(I) = J;
                swap_indexes(J) = I;
                opt_solution = previous_solution(swap_indexes);
                
                % calc. fitness of new solution
                opt_fitness = fitness(opt_solution,distance_matrix);
                
                % To do: Check if the newly found solution is better.                               
                if (opt_fitness < best_fitness_yet)
                    % To do: Update solution, and store new better fitness score.
                    solution = opt_solution;
                    best_fitness_yet = opt_fitness;
                    
                    % Leave the next three lines as they are
                    disp(sprintf('Local search: %f',best_fitness_yet));
                    run=1;
                    break;
                end

          
            end
            if(run==1), break; end
        end
        if(run==1), break; end
    end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function f=fitness(solution,distance_matrix)
%% function f=fitness(solution,distance_matrix);
%
% Calculates the length of a solution given the distance matrix between all
% cities. 
f=0;
if(nargin<3) best=10;
end
for I=1:length(solution)-1
    f=f+distance_matrix(solution(I),solution(I+1));
end
f=f+distance_matrix(solution(1),solution(length(solution)));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function citylist=delete_entry(citylist,city)
%% function citylist=delete_entry(citylist,city)
% The city 'city' is deleted from a list of cities 'citylist'
citylist=citylist(find(citylist~=city)); % find returnts the indices of the elements in citylist not equal to the city



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function d=distance(X1,X2)
%% function d=distance(X1,X2)
% Calculates the distance between two vectors 'X1' and 'X2'. Vectors
% contain x and y coordinates, and can have multiple rows. 
d=sqrt(sum((X1-X2).^2,2));
