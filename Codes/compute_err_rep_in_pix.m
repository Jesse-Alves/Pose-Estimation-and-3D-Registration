function res = compute_err_rep_in_pix(Rcam_obj_est, tcam_obj_est, object, pts_im, K)

% Question 2.2
% Compute reprojection error in the image

	N = size(object,2);

    pts_in_cam_frame = Rcam_obj_est*object + tcam_obj_est;
    
    pts_in_unit_plane = pts_in_cam_frame./pts_in_cam_frame(3,:);
    
    pts_im_rep = K*pts_in_unit_plane;

    mat_err_pts = pts_im - pts_im_rep;
        

% MAError
pts_dist = sqrt(mat_err_pts(1,:).^2 + mat_err_pts(2,:).^2)

res = mean(pts_dist);
    
end
