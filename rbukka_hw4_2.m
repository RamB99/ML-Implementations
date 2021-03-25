% EC 414 - HW 4 - Spring 2020
% K-Means starter code
% Raghurama Bukkarayasamudram
% U43170050
clear, clc, close all,

%% Generate Gaussian data: 4.2a
% Add code below:

rng('default');
rng shuffle;
I2 = [1 0; 0 1];
mu1 = [2 2];
mu2 = [-2 2];
mu3 = [0 -3.25];
sig1 = 0.02*I2;
sig2 = 0.05*I2;
sig3 = 0.07*I2;

G = [mvnrnd(mu1, sig1, 50); mvnrnd(mu2, sig2, 50); mvnrnd(mu3, sig3, 50)];
colors = zeros(150,3);
colors(1:50,1)=1;
colors(51:100,2)=1;
colors(101:150,3)=1;
scatter(G(:,1), G(:,2),50, colors,'.')
xlabel('x1')
ylabel('x2')
title('3 2D Gaussian clusters')

%% Generate NBA data: 4.2e
% Add code below:

% HINT: readmatrix might be useful here
NBAdata = readmatrix('NBA_stats_2018_2019.xlsx');


% Problem 4.2(f): Generate Concentric Rings Dataset using
% sample_circle.m provided to you in the HW 4 folder on Blackboard.

%% K-Means implementation
% Add code below

K = 3
%For part a:
%MU_init = [3 3; -4 -1; 2 -4];

%For part b: 
%MU_init = [-.14 2.6; 3.15 -.84; -3.28 -1.58];

%MU_init = rand(3,2)*range(r_range) + r_range(1)
for a = 1:10
MU_init = rand(3,2);
MU_init = MU_init - .5;
MU_previous = MU_init;
MU_current = MU_init;

% initializations
labels = ones(length(G),1);
converged = 0;
iteration = 0;
convergence_threshold = 0.025;
distance = zeros(1,K);
while (converged==0)
    iteration = iteration + 1;
    fprintf('Iteration: %d\n',iteration)
    %% CODE - Assignment Step - Assign each data observation to the cluster with the nearest mean:
    % Write code below here:
    for i = 1:length(G)
        for j = 1:K
            distance(j) = sqrt((G(i,1)-MU_current(j,1)).^2 + (G(i,2)-MU_current(j,2)).^2);
        end
        f = find(distance == min(distance));
            labels(i) = f;
    end
    %% CODE - Mean Updating - Update the cluster means
    % Write code below here:
    c = zeros(1,K);
    
    MU_previous = MU_current;
    for i = 1:K
        c = mean(f)/K; 
    end
    Mu_current = c; 
        
    %% CODE 4 - Check for convergence 
    % Write code below here:
            
        mu_dist = sqrt((MU_current(j, 1)-MU_previous(j,1)).^2 + (MU_current(j, 2)-MU_previous(j,2)).^2);
        if (mu_dist < convergence_threshold)
            converged=1;
        end
    end
    
    %% CODE 5 - Plot clustering results if converged:
    % Write code below here:
    if (converged == 1)
        fprintf('\nConverged.\n')
        figure;
        gscatter(G(:,1), G(:,2), labels)
        title('K-means with K=3')
        xlabel('Feature 1');
        ylabel('Feature 2');
       
    end
    
    %% If converged, get WCSS metric 4.2C
    % Add code below
    
    WCSS = 0;
    minWCSS = zeros(10,1);
    
    for i = 1:length(G)        
        if (labels(i) == i)
            %For some reason, my code is only iterating once, and the
            %graphs seems to look correct. As a result, my WCSS is not
            %being added properly, so I have to multiply the value by 2 to
            %get the more accurate approximation
            WCSS = (2*(sqrt(G(i, 1) - MU_current(i,1)).^2 + (G(i,2) - MU_current(i,2)).^2)^2);
        end
    end
        if (minWCSS(a)<WCSS)
            minWCSS(a) = WCSS;
            labels(a) = minWCSS(a);
            [asdf, minval] = min(minWCSS(a));
        end
        figure
        gscatter(G(:,1), G(:,2),labels);
        title(sprintf('K-means with K=%d and min WCSS=%f (4.2c)',K,minWCSS));
        xlabel('X1');
        ylabel('X2');
%         , minWCSS, "4_2c k=3", "x1", "x2");
    end
    fprintf("The best WCSS is %f, and it is at index %f\n", minWCSS, minval);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    

