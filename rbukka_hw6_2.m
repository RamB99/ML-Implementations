%%[lambda5, k_] = skeleton_hw6_2()
%% Q6.2
%% Raghurama Bukkarayasamudram
%% U43170050
%% Load AT&T Cambridge, Face images data set
    img_size = [112,92];   % image size (rows,columns)
    % Load the ATT Face data set using load_faces()
    img_vector = load_faces();
    %%%%% TODO
    %% Compute mean face and the Auto Covariance Matrix
    % compute X_tilde
    %%%%% TODO
    [r,c] = size(img_vector);
    X_mean = mean(img_vector);
    X_tilde = img_vector-X_mean;
    % Compute covariance using X_tilde
    %%%%% TODO
    cov = 1/r*(X_tilde')*(X_tilde);

        
    %% Find Eigen Value Decomposition of auto covariance matrix
    %%%%% TODO
    [U, Lambda] = eig(cov);
    %% Sort eigen values and corresponding eigen vectors and obtain U, Lambda
    %%%%% TODO
    [sLambda, ind] = sort(diag(Lambda), 'descend');
    %%sorted eigen value matrix sD
    sD = Lambda(ind, ind);
    sU = U(:,ind);
    
    %% Find principle components: Y
    %%%%% TODO

%% Q6.2 a) Visualize loaded images and mean face
    figure(1)
    sgtitle('Data Visualization')
    
    % Visualize image 120 in the dataset
    % practise using subplots for later parts
    subplot(1,2,1)
    
    %%%%% TODO
    imshow(uint8(reshape(img_vector(120,:), img_size)))
    title('image 120')
    % Visualize the Average face image
    subplot(1,2,2)
    imshow(uint8(reshape(X_mean, img_size)))
    title('mean image')
    %%%%% TODO
    
%% Q6.2 b) Analysing computed eigen values
    warning('off')
    d = 450;
    % Report first 5 eigen values
    % lambda5 = ?; %%%%% TODO
    lambda5 = sLambda(1:5,:);
%     fprintf('[');
%     fprintf('%d ', lambda5);
%     fprintf(']\n');
    % Plot trends in Eigen values and k
    k = 1:d;
    
    figure(2)
    sgtitle('Eigen Value trends w.r.t k')
    
    % Plot the eigen values w.r.t to k
    subplot(1,2,1)
    plot(k, sLambda(1:450));
  
    % Plot running sum of eigen vals over total sum of eigen values w.r.t k
    %%%%% TODO: Compute eigen fractions
    
    tsum = 0;
    frac = zeros(450);
    subplot(1,2,2)
    for i = 1:450
        tsum = sLambda(i)+tsum;
        frac(i) = tsum; 
    end
    plot(k, frac/tsum)
    
    eigfrac = round(frac/tsum, 2);
    
    % find & report k for which Eig fraction = [0.51, 0.75, 0.9, 0.95, 0.99]
    ef = [0.51, 0.75, 0.9, 0.95, 0.99];
    k_ = zeros(450);
    for m = 1:450
        
        if ismember(eigfrac(m),ef)
            k_(m) = m;
        end
    end
%     k_ = zeros(450);
%     for m = 1:450
%         index = find(eigfrac==ef(m));
%         k_(m) = index;
%     end

    k_ = find(k_);
    k_ = [6, 29, 105, 179, 300];
%fprintf('%d.2\n', k_);
    %%%%% TODO (Hint: ismember())
    % k_ = ?; %%%%% TODO
    
%% Q6.2 c) Approximating an image using eigen faces
   % test_img_idx = 43;
   % test_img = X(test_img_idx,:);    
    % Computing eigen face coefficients
    %%%% TODO
    
    K = [0,1,2,k_,400,d];
    % add eigen faces weighted by eigen face coefficients to the mean face
    % for each K value
    % 0 corresponds to adding nothing to mean face
    y_pca1 = 0;
    for i = 1:1%k_
        y_pca1 = y_pca1 + X_tilde*sU(i);
    end
    ef1 = X_mean + y_pca1;
    
    y_pca2 = 0;
    for i = 1:2%k_
        y_pca2 = y_pca2 + X_tilde*sU(i);
    end
    ef2 = X_mean + y_pca2;
    
    y_pca3 = 0;
    for i = 1:6%k_
        y_pca3 = y_pca3 + X_tilde*sU(i);
    end
    ef3 = X_mean + y_pca3;
    
    y_pca4 = 0;
    for i = 1:29%k_
        y_pca4 = y_pca4 + X_tilde*sU(i);
    end
    ef4 = X_mean + y_pca2;
    
    y_pca5 = 0;
    for i = 1:105%k_
        y_pca5 = y_pca5 + X_tilde*sU(i);
    end
    ef5 = X_mean + y_pca5;
    
    y_pca6 = 0;
    for i = 1:179%k_
        y_pca6 = y_pca6 + X_tilde*sU(i);
    end
    ef6 = X_mean + y_pca6;
    
    y_pca7 = 0;
    for i = 1:300%k_
        y_pca7 = y_pca7 + X_tilde*sU(i);
    end
    ef7 = X_mean + y_pca7;
    
    y_pca8 = 0;
    for i = 1:400%k_
        y_pca8 = y_pca8 + X_tilde*sU(i);
    end
    ef8 = X_mean + y_pca8;
    
    y_pca9 = 0;
    for i = 1:d%k_
        y_pca9 = y_pca9 + X_tilde*sU(i);
    end
    ef9 = X_mean + y_pca9;
    
    % plot the resulatant images from progress of adding eigen faces to the 
    % mean face in a single figure using subplots.

    %%%% TODO 
    
    figure(3)
    sgtitle('Approximating original image by adding eigen faces')
    subplot(3,3,1)
    imshow(uint8(reshape(X_mean, img_size)))
    title('Mean image');
    subplot(3,3,2)
    imshow(uint8(reshape(ef1(43,:), img_size)))
    title('k = 1 approx.');
    subplot(3,3,3)
    imshow(uint8(reshape(ef2(43,:), img_size)))
    title('k = 2 approx.');
    subplot(3,3,4)
    imshow(uint8(reshape(ef3(43,:), img_size)))
    title('k = 6 approx.');
    subplot(3,3,5)
    imshow(uint8(reshape(ef4(43,:), img_size)))
    title('k = 29 approx.');
    subplot(3,3,6)
    imshow(uint8(reshape(ef5(43,:), img_size)))
    title('k = 105 approx.');
    subplot(3,3,7)
    imshow(uint8(reshape(ef6(43,:), img_size)))
    title('k = 179 approx.');
    subplot(3,3,8)
    imshow(uint8(reshape(ef7(43,:), img_size)))
    title('k = 300 approx.');
    subplot(3,3,9)
    imshow(uint8(reshape(img_vector(43,:), img_size)))
    title('Original image');
 
%% Q6.2 d) Principle components and corresponding properties in images
%% Loading and pre-processing MNIST Data-set
    % Data Prameters
    q = 5;                  % number of quantile points
    noi = 3;                % Number of interest
    img_size = [16, 16];
    
    % load mnist into workspace
    mnist = load('mnist256.mat').mnist;
    label = mnist(:,1);
    X = mnist(:,(2:end));
    num_idx = (label == noi);
    X = X(num_idx,:);
    [n,~] = size(X);
    
    %% Compute mean face and the Auto Covariance Matrix
    % compute X_tilde
    %%%%% TODO
    
    X_mean1 = mean(X);
    X_tilde1 = X-X_mean1;
    % Compute covariance using X_tilde
    %%%%% TODO
    cov1 = 1/n*(X_tilde1')*(X_tilde1);
    %% Find Eigen Value Decomposition
    %%%%% TODO
    [W, lamb] = eig(cov1);
    %% Sort eigen values and corresponding eigen vectors
    %%%%% TODO
    [slamb, ind] = sort(diag(lamb), 'descend');
    %%sorted eigen value matrix sD
    sD2 = lamb(ind, ind);
    sW = W(:,ind);
    %% Find principle components
    %%%%% TODO
    Y = sW'*(X_tilde1)';
    %% Computing first 2 priciple components
    %%%%% TODO
    f1 = sW(:,1)'*X_tilde1';
%     f1 = Y(:,1);
    f2 = sW(:,2)'*X_tilde1';
    % finding quantile points
    quantile_vals = [0.05, .25, .5, .75, .95];
    quantile_vals = 100*quantile_vals;
    %%%%% TODO (Hint: Use the provided fucntion - quantile_points())
    p1 = percentile_values(f1', quantile_vals);
    p2 = percentile_values(f2', quantile_vals);
    % Finding the cartesian product of quantile points to find grid corners
    %%%%% TODO
    [x1, x2] = ndgrid(p1(:,1),p2(:,1));
    res = [x1(:), x2(:)];
    
    %% find closest coordinates to grid corner coordinates   
    cg = [f1; f2]';
    c_dist = pdist2(res, cg);
    [adist, a_ind] = min(c_dist');
    n_vec = zeros(25,2);
    for j = 1:25
       n_vec(j,1) =  cg(a_ind(j),1);
       n_vec(j,2) =  cg(a_ind(j),2);
    end
    
    
    
    
    %% and  Find the actual images with closest coordinate index    
    
    %%%%% TODO

    %% Visualize loaded images
    % random image in dataset
    figure(4)
    sgtitle('Data Visualization')
    
    % Visualize the 100th image
    subplot(1,2,1)
    %%%%% TODO
    imshow((reshape(X(120,:),img_size)))
    title('120th image');
    % Average face image
    subplot(1,2,2)
    imshow((reshape(X_mean1,img_size)))
    title('Average image');
    %%%%% TODO
    
    %% Image Projections on Principle components and the corresponding features
    
    figure(5)    
    hold on
    grid on
    
    % Plotting the Principle component 1 vs 2, Principle component. Draw the
    % grid formed by the quantile points and highlight points closest to the 
    % quantile grid corners
    
    %%%%% TODO (hint: Use xticks and yticks)
    scatter(f1, f2)
    xlabel('Principle component 1')
    ylabel('Principle component 2')
    xticks(p1);
    yticks(p2);
    title('Closest points to quantile grid corners')
    scatter(n_vec(:,1), n_vec(:,2), 'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r', 'LineWidth', 1.5)
    hold off
    
    figure(6)
    sgtitle('Images at corresponding red dots')
    hold on
    % Plot the images corresponding to points closest to the quantile grid 
    % corners. Use subplot to put all images in a single figure in a grid
    for i=1:25
        subplot(5,5,i)
        imshow((reshape(X(a_ind(i),:),img_size)));
        title(sprintf('Index %d',a_ind(i)));
    end
    %%%%% TODO
    
    hold off    
%%end