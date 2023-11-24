function [Rcam_obj_est, tcam_obj_est, Rnon_ortho] = Linear_PnP(object, pts_im, K)

nb_pts = size(object, 2);

% Linear solving method using translation and rotation matrix coefficients as unknowns

m = inv(K)*pts_im;

% Writing the estimation problem as a system of linear equations in form
% AX = 0

for ii = 1:2:(2*nb_pts)
    index = idivide(int32(ii),2,'ceil');
    A(ii,:) = [object(:,index)' zeros(1,3) -m(1,index)*object(:,index)' 1 0 -m(1,index)];
    A(ii+1,:) = [zeros(1,3) object(:,index)' -m(2,index)*object(:,index)' 0 1 -m(2,index)];
end

% A is a 12x12 matrix with rank = 11. Since AX = 0, X lies within the kernel of A
% and consequently Ax = 0 for any vector x that lives in the subspace spanned by
% the last column of V, meaning it lies in the null space.

[U,S,V] = svd(A);

% Extracting rotation and translation components of the pose
R = [V(1:3,12)';
     V(4:6,12)';
     V(7:9,12)'];

t = V(10:12,12);

% Scaling solution so that det=1 (note that it does not impose orthogonality
% constraints) 

scaling_factor = nthroot(1/det(R),3);

tcam_obj_est = scaling_factor*t;
Rnon_ortho = scaling_factor*R;

% Enforcing orthogonality constraints

[U,S,V] = svd(R);

if (det(U*V') < 0)
    Rcam_obj_est = U*[1 0 0; 0 1 0; 0 0 -1]*V';
else
    Rcam_obj_est = U*V';
end

