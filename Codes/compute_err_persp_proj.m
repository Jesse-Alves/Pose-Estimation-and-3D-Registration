function res = compute_err_persp_proj(x, pts_obj, pts_cam)

persistent pts_obj_loc pts_cam_loc N

if nargin>1
    % Initialization of constant parameters during optimization
	pts_obj_loc = pts_obj;
	pts_cam_loc = pts_cam;
	N = size(pts_obj,2);
	res = 0;

else

    % Computation of the error for the current estimated pose x with
    % Euler angles

    % First, create a homogeneous transformation matrix from angles
    % psi, theta and phi
    R_psi = [cos(x(1)) -sin(x(1)) 0; sin(x(1)) cos(x(1)) 0; 0 0 1];
    R_theta = [1 0 0; 0 cos(x(2)) -sin(x(2)); 0 sin(x(2)) cos(x(2))];
    R_phi = [cos(x(3)) -sin(x(3)) 0; sin(x(3)) cos(x(3)) 0; 0 0 1];
    R = R_psi*R_theta*R_phi; 
    t = [x(4);x(5);x(6)];
    est_pose = [R t; 0 0 0 1];

    % Second, use said transformation matrix to project the object points
    % onto the image plane
    posed_obj = est_pose*[pts_obj_loc;ones(1,6)];
    est_pts_cam = posed_obj./posed_obj(3,:);
    est_pts_cam(4,:) = [];

    % Finally, the error to be minimized is the difference between said 
    % reprojected points and the reference points.
    error = (pts_cam_loc - est_pts_cam);
    res = error;

    % If fminunc is used instead of lsqnonlin, the output of the function
    % needs to be the sum of the squared norms of each error vector:
    % for ii = 1 : N
    %   res_norm_sq(ii) = norm(error(:,ii))^2;
    % end
    % res = sum(res_norm_sq);
    
end

