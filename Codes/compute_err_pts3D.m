function err_pts_3D = compute_err_pts3D(Rcam_obj_est, tcam_obj_est, object, list_pts_in_cam)

% Question 2.1
% Compute the 3D reconstruction error on 3D points expressed in the camera frame
% A function computing the MAE on the 3D positions of the reconstructed points. The function 
% will take as input arguments the estimated pose, the structure of the object and the list of exact 3D 
% points (ground truth).

nb_pts = size(object, 2);

pts_cam_est = Rcam_obj_est* object + tcam_obj_est;
        
% Matrix containing 3D errors on reconstructed points
mat_err_pts = list_pts_in_cam - pts_cam_est;

% MAE error
pts_dist = sqrt(mat_err_pts(1,:).^2 + mat_err_pts(2,:).^2 + mat_err_pts(3,:).^2)

err_pts_3D = mean(pts_dist);

end

