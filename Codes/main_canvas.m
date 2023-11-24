%% Canvas for pose estimation tutorial
% Submitted on November 6th, 2022. Made by HealthTech M1 students:
% Jessé Alves
% João Luiz Machado Junior
% Luis Villamarin

clear all;
close all;

addpath EPnP;

% Loading data
load data_tutorial_PnP;
% pause;

nb_config_test = numel(list_T);
nb_pts = size(object,2);

trans_err = [];
rot_err = [];
err_pts_3D = [];

list_methodes_testees = [1,2,3,4]; % to modify depending on the methods you want to test
% 1 : analytic PnP method (provided)
% 2 : linear method BEFORE orthogonality constraints enforcement
% 3 : linear method AFTER orthogonality constraints enforcement
% 4 : numerical optimization

noise_ampl = [0, 0.1, 0.5, 1, 2];
nb_noise = length(noise_ampl);

%% Question 1.1
% Compute perfect image points (without noise) for each configuration to test
list_pts_im_perf = cell(1,271);

for kk=1:nb_config_test
    
    % First, compute the change of frame from object coordinates to
    % camera frame by multiplying each point with a configuration (pose)
    % from each of the 271 samples.

    poses = list_T{kk}*[object; ones(1,6)];

    % Then, remove fourth row and perform perspective projection
    % on a unit distance plane

    poses(4,:) = [];
    perspective_projection = poses./poses(3,:);
     
    % Finally, conclude image formation by applying
    % Intrinsic Parameters Matrix
    list_pts_im_perf(1,kk) = mat2cell(K*perspective_projection,3,6);

end

%% Question 1.2

for ii=1:nb_noise 
    for kk=1:nb_config_test 
        
        ang_rand(ii,:) = 2*pi*rand(1,6);
        direction = [cos(ang_rand(ii,:)); sin(ang_rand(ii,:))];
    
        current_noises = [noise_ampl(ii)*direction; zeros(1,6)];
        mat_pts_im{ii}{kk} = cell2mat(list_pts_im_perf(kk)) + current_noises;
        
    end
end


%% Question 2.3
for ii=1:nb_noise

    tic
    for kk=1:nb_config_test
    
        % Exact values (= ground truth) for the tested configuration
        tcam_obj_ref = list_tcam_obj{kk}; 
		Rcam_obj_ref = list_Rcam_obj{kk};	
        
        pts_im = mat_pts_im{ii}{kk};
        
		% Results from PnP method
        [Rcam_obj_est,tcam_obj_est]=efficient_pnp([object', ones(nb_pts,1)], [pts_im', ones(nb_pts,1)],K)
		%[Rcam_obj_est,tcam_obj_est] =efficient_pnp_gauss([object', ones(nb_pts,1)], [pts_im', ones(nb_pts,1)],K)
        
        pos_err{1}(ii,kk,1) = tcam_obj_ref(1) - tcam_obj_est(1);
		pos_err{1}(ii,kk,2) = tcam_obj_ref(2) - tcam_obj_est(2);
		pos_err{1}(ii,kk,3) = tcam_obj_ref(3) - tcam_obj_est(3);
		trans_err{1}(ii,kk) = norm(tcam_obj_ref - tcam_obj_est);
		
        [theta,u] = r2thetau(Rcam_obj_ref*Rcam_obj_est');
		rot_err{1}(ii,kk) = abs(theta);
		
        err_pts_3D{1}(ii,kk) = compute_err_pts3D(Rcam_obj_est, tcam_obj_est, object, list_pts_in_cam{kk});
        err_rep{1}(ii,kk) = compute_err_rep_in_pix(Rcam_obj_est, tcam_obj_est, object, pts_im, K);
        	
    end
    
     analytic_duration(ii) = toc;
    
     figure;
     plot3Derrors( list_trans(:,list_ind_no_rot), err_pts_3D{1}(ii,list_ind_no_rot))
     strtitle = sprintf('Position errors of 3D points for analytical method for %.2f pix. noise', noise_ampl(ii)); 
     title(strtitle);
    
     figure;
     plot3Derrors( list_trans(:,list_ind_no_rot), trans_err{1}(ii,list_ind_no_rot))
     strtitle = sprintf('translation error for analytical method for %.2f pix. noise', noise_ampl(ii)); 
     title(strtitle);
    
     figure;
      plot3Derrors( list_trans(:,list_ind_no_rot), err_rep{1}(ii,list_ind_no_rot))
     strtitle = sprintf('image reprojection error for analytical method for %.2f pix. noise', noise_ampl(ii)); 
     title(strtitle);
      
     % Comparison of translation errors along individual axes
     figure;
     plot(pos_err{1}(ii,:,1), 'r');
     hold on;
     plot(pos_err{1}(ii,:,2), 'g');
     plot(pos_err{1}(ii,:,3), 'b');
     legend('x', 'y', 'z');
     strtitle = sprintf('translation errors for analytical method for %.2f pix. noise', noise_ampl(ii)); 
     title(strtitle);
     
end
analytic_duration 

% Statistics (median and max errors) per noise level
for jj=1:nb_noise	
	trans_err_cons = trans_err{1}(jj,:);
	trans_err_filt = trans_err_cons(find(trans_err_cons ~= inf));	
				
	%rot_err_cons = rot_err{1}(jj,:);
	%rot_err_filt = rot_err_cons(find(rot_err_cons ~= inf));	
		
    points_err_cons = err_pts_3D{1}(jj,:);
    points_err_filt = points_err_cons(find(points_err_cons ~= inf));
        
    im_err_cons = err_rep{1}(jj,:);
    im_err_filt = im_err_cons(find(im_err_cons ~= inf));
        
        
    trans_err_med{1}(jj) = median(trans_err_filt,2); 
	%rot_err_med{1}(jj) = median(rot_err_filt,2); 
	
	trans_err_max{1}(jj) = max(trans_err_filt,[],2); 
	%rot_err_max{ii}(jj) = max(rot_err_filt,[],2); 
	
    points_err_med{1}(jj) = median(points_err_filt,2);
    points_err_max{1}(jj) = max(points_err_filt,[],2);
        
    im_err_med{1}(jj) = median(im_err_filt,2);
    im_err_max{1}(jj) = max(im_err_filt,[],2);
        
end

     
 %% Question 3
 for ii=1:nb_noise
   tic
   for kk=1:nb_config_test
        tcam_obj_ref = list_tcam_obj{kk}; 
		Rcam_obj_ref = list_Rcam_obj{kk};
	
        pts_im = mat_pts_im{ii}{kk};
       
        % Linear solution for R and t 
        [Rcam_obj_est, tcam_obj_est, Rnon_ortho] = Linear_PnP(object, pts_im, K);
    	
        trans_err{2}(ii,kk) = norm(tcam_obj_ref - tcam_obj_est);
    	trans_err{3}(ii,kk) = norm(tcam_obj_ref - tcam_obj_est);
    	
        [theta,u] = r2thetau(Rcam_obj_ref*Rnon_ortho');
    	rot_err{2}(ii,kk) = abs(theta);
       
        err_rep{2}(ii,kk) = compute_err_rep_in_pix(Rnon_ortho, tcam_obj_est, object, pts_im, K);
        err_rep{3}(ii,kk) = compute_err_rep_in_pix(Rcam_obj_est, tcam_obj_est, object, pts_im, K);
       
        err_pts_3D{2}(ii,kk) = compute_err_pts3D(Rnon_ortho, tcam_obj_est, object, list_pts_in_cam{kk});
        
        err_pts_3D{3}(ii,kk) = compute_err_pts3D(Rcam_obj_est, tcam_obj_est, object, list_pts_in_cam{kk});
       
        %pause;
    end
  	linear_duration(ii) = toc;
    
     figure;
     plot3Derrors( list_trans(:,list_ind_no_rot), err_pts_3D{3}(ii,list_ind_no_rot))
      strtitle = sprintf('Position errors of 3D points for linear method for %.2f pix. noise', noise_ampl(ii)); 
     title(strtitle);
    
     figure;
     plot3Derrors( list_trans(:,list_ind_no_rot), trans_err{3}(ii,list_ind_no_rot))
     strtitle = sprintf('translation error for linear method for %.2f pix. noise', noise_ampl(ii)); 
     title(strtitle);
    
     figure;
      plot3Derrors( list_trans(:,list_ind_no_rot), err_rep{3}(ii,list_ind_no_rot))
      strtitle = sprintf('image reprojection error for linear method for %.2f pix. noise', noise_ampl(ii)); 
     title(strtitle);
       
 end
 
 linear_duration
 
% Statistics (median and mex errors) per noise level

for jj=1:nb_noise	
	trans_err_cons = trans_err{2}(jj,:);
	trans_err_filt = trans_err_cons(find(trans_err_cons ~= inf));	
				
	%rot_err_cons = rot_err{2}(jj,:);
	%rot_err_filt = rot_err_cons(find(rot_err_cons ~= inf));	
		
    points_err_cons = err_pts_3D{2}(jj,:);
    points_err_filt = points_err_cons(find(points_err_cons ~= inf));
        
    im_err_cons = err_rep{2}(jj,:);
    im_err_filt = im_err_cons(find(im_err_cons ~= inf));
        
        
    trans_err_med{2}(jj) = median(trans_err_filt,2); 
	%rot_err_med{2}(jj) = median(rot_err_filt,2); 
	
	trans_err_max{2}(jj) = max(trans_err_filt,[],2); 
	%rot_err_max{2}(jj) = max(rot_err_filt,[],2); 
	
    points_err_med{2}(jj) = median(points_err_filt,2);
    points_err_max{2}(jj) = max(points_err_filt,[],2);
        
    im_err_med{2}(jj) = median(im_err_filt,2);
    im_err_max{2}(jj) = max(im_err_filt,[],2);
        
end

for jj=1:nb_noise	
	trans_err_cons = trans_err{3}(jj,:);
	trans_err_filt = trans_err_cons(find(trans_err_cons ~= inf));	
				
	%rot_err_cons = rot_err{2}(jj,:);
	%rot_err_filt = rot_err_cons(find(rot_err_cons ~= inf));	
		
    points_err_cons = err_pts_3D{3}(jj,:);
    points_err_filt = points_err_cons(find(points_err_cons ~= inf));
        
    im_err_cons = err_rep{3}(jj,:);
    im_err_filt = im_err_cons(find(im_err_cons ~= inf));
        
        
    trans_err_med{3}(jj) = median(trans_err_filt,2); 
	%rot_err_med{2}(jj) = median(rot_err_filt,2); 
	
	trans_err_max{3}(jj) = max(trans_err_filt,[],2); 
	%rot_err_max{2}(jj) = max(rot_err_filt,[],2); 
	
    points_err_med{3}(jj) = median(points_err_filt,2);
    points_err_max{3}(jj) = max(points_err_filt,[],2);
        
    im_err_med{3}(jj) = median(im_err_filt,2);
    im_err_max{3}(jj) = max(im_err_filt,[],2);
        
end

%% Question 4
for ii=1:nb_noise   
    tic
    for kk=1:nb_config_test
        tcam_obj_ref = list_tcam_obj{kk}; 
		Rcam_obj_ref = list_Rcam_obj{kk};
	
        % Compute image points in normalized image plane
        pts_im = mat_pts_im{ii}{kk};
        pts_cam = inv(K)*pts_im;
        
        % Numerical minimization
        % Initialization of objective function to minimize
		compute_err_persp_proj(0, object, pts_cam);
	
        % Initialization of pose
        % <<The configuration used to initialize the optimization will
        % typically be chosen in the middle of the workspace and
        % will be the same for all resolutions>>
        t_ini = sum(object,2)/size(object,2);
        psi = 0;
        theta = 0;
        phi = 0;
        x_ini = [psi; theta; phi; t_ini];

        fprintf('initial error\n');
		res = compute_err_persp_proj(x_ini);
		
        % Optimization using fminunc function
        % x = fminunc(@compute_err_persp_proj,x_ini);
	   
        % Optimization using lsqnonlin function
        options = optimoptions('lsqnonlin', 'Algorithm','levenberg-marquardt');
        x = lsqnonlin(@compute_err_persp_proj,x_ini);	

        % Re-composing the rotation matrix through the Euler angles
    	R_psi = [cos(x(1)) -sin(x(1)) 0; sin(x(1)) cos(x(1)) 0; 0 0 1];
        R_theta = [1 0 0; 0 cos(x(2)) -sin(x(2)); 0 sin(x(2)) cos(x(2))];
        R_phi = [cos(x(3)) -sin(x(3)) 0; sin(x(3)) cos(x(3)) 0; 0 0 1];        		
    
        % Extraction of Rcam_obj_est and tcam_obj
        Rcam_obj_est = R_psi*R_theta*R_phi; 
		tcam_obj_est = x(4:6);
    
        trans_err{4}(ii,kk) = norm(tcam_obj_ref - tcam_obj_est);
		err_rep{4}(ii,kk) = compute_err_rep_in_pix(Rcam_obj_est, tcam_obj_est, object, pts_im, K);
     
        err_pts_3D{4}(ii,kk) = compute_err_pts3D(Rcam_obj_est, tcam_obj_est, object, list_pts_in_cam{kk});
       
        
    
    optim_duration(ii) = toc;

    end
   %pause;
 
     figure;
     plot3Derrors( list_trans(:,list_ind_no_rot), err_pts_3D{4}(ii,list_ind_no_rot))
     strtitle = sprintf('Position errors of 3D points for optimization method for %.2f pix. noise', noise_ampl(ii)); 
     title(strtitle);
    
     figure;
     plot3Derrors( list_trans(:,list_ind_no_rot), trans_err{4}(ii,list_ind_no_rot))
    strtitle = sprintf('translation error for optimization method for %.2f pix. noise', noise_ampl(ii)); 
     title(strtitle);
    
     figure;
      plot3Derrors( list_trans(:,list_ind_no_rot), err_rep{4}(ii,list_ind_no_rot))
        strtitle = sprintf('image reprojection error for optimization method for %.2f pix. noise', noise_ampl(ii)); 
     title(strtitle);
      
 end
optim_duration

% Statistics (median and mex errors) per noise level

for jj=1:nb_noise	
	trans_err_cons = trans_err{4}(jj,:);
	trans_err_filt = trans_err_cons(find(trans_err_cons ~= inf));	
				
	%rot_err_cons = rot_err{4}(jj,:);
	%rot_err_filt = rot_err_cons(find(rot_err_cons ~= inf));	
		
    points_err_cons = err_pts_3D{4}(jj,:);
    points_err_filt = points_err_cons(find(points_err_cons ~= inf));
        
    im_err_cons = err_rep{4}(jj,:);
    im_err_filt = im_err_cons(find(im_err_cons ~= inf));
        
        
    trans_err_med{4}(jj) = median(trans_err_filt,2); 
	%rot_err_med{4}(jj) = median(rot_err_filt,2); 
	
	trans_err_max{4}(jj) = max(trans_err_filt,[],2); 
	%rot_err_max{4}(jj) = max(rot_err_filt,[],2); 
	
    points_err_med{4}(jj) = median(points_err_filt,2);
    points_err_max{4}(jj) = max(points_err_filt,[],2);
        
    im_err_med{4}(jj) = median(im_err_filt,2);
    im_err_max{4}(jj) = max(im_err_filt,[],2);
        
end
     
     
col = ['r', 'b', 'g', 'k', 'm', 'c', 'y'];
figure;
hold on;
P = [];
for kk=list_methodes_testees
	color = sprintf('%s*-', col(kk));
	P = [P plot(noise_ampl, trans_err_med{kk}, color)];
end
legend('Analytic', 'linear wo norm', 'linear', 'optimisation', 'Location', 'NorthWest');
xlabel('amplitude of image noise');
ylabel('translation error (mm)')
title('Mediane translation error');
set(P, 'linewidth', 2);


figure;
hold on;
P = [];
for kk=list_methodes_testees
	color = sprintf('%s*-', col(kk));
	P = [P plot(noise_ampl, trans_err_max{kk}, color)];
end
legend('Analytic', 'linear wo norm', 'linear', 'optimisation', 'Location', 'NorthWest');
xlabel('amplitude of image noise');
ylabel('translation error (mm)')
title('Max translation error');
set(P, 'linewidth', 2);

 
figure;
hold on;
P = [];
for kk=list_methodes_testees
	color = sprintf('%s*-', col(kk));
	P = [P plot(noise_ampl, points_err_med{kk}, color)];
end
legend('Analytic', 'linear wo norm', 'linear', 'optimisation', 'Location', 'NorthWest');
xlabel('amplitude of image noise');
ylabel('3D points error (mm)')
title('Mediane error on 3D points');
set(P, 'linewidth', 2);


figure;
hold on;
P = [];
for kk=list_methodes_testees
	color = sprintf('%s*-', col(kk));
	P = [P plot(noise_ampl, points_err_max{kk}, color)];
end
legend('Analytic', 'linear wo norm', 'linear', 'optimisation', 'Location', 'NorthWest');
xlabel('amplitude of image noise');
ylabel('3D points error (mm)')
title('Max error on 3D points');
set(P, 'linewidth', 2);


figure;
hold on;
P = [];
for kk=list_methodes_testees
	color = sprintf('%s*-', col(kk));
	P = [P plot(noise_ampl, im_err_med{kk}, color)];
end
legend('Analytic', 'linear wo norm', 'linear', 'optimisation', 'Location', 'NorthWest');
xlabel('amplitude of image noise');
ylabel('image points error (pix)')
title('Mediane error in image');
set(P, 'linewidth', 2);


figure;
hold on;
P = [];
for kk=list_methodes_testees
	color = sprintf('%s*-', col(kk));
	P = [P plot(noise_ampl, im_err_max{kk}, color)];
end
legend('Analytic', 'linear wo norm', 'linear', 'optimisation', 'Location', 'NorthWest');
xlabel('amplitude of image noise');
ylabel('image points error (pix)')
title('Max error in image');
set(P, 'linewidth', 2);
