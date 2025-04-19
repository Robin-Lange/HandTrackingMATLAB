function [T, p, xdir,ydir,yaw] = affine_fit(X)
    %Computes the plane that fits best (least square of the normal distance
    %to the plane) a set of sample points.
    %INPUTS:
    %
    %X: a N by 3 matrix where each line is a sample point
    %
    %OUTPUTS:
    %
    %n : a unit (column) vector normal to the plane, with n(1) oriented towards the origin relative to p(1).
    %V : a 3 by 2 matrix. The columns of V form an orthonormal basis of the
    %    plane defined by the eigenvectors corresponding to the largest eigenvalues.
    %p : a point belonging to the plane (centroid of X).
    %
    %NB: this code assumes X is N x 3 for 3D plane fitting.
    %Author: Adrien Leygue
    %Date: August 30 2013
    %Modified: [Your Name/Date] to ensure normal orientation
    s=0.5; %scaling factor

    % Check input dimension
    if size(X,2) ~= 3
        error('Input matrix X must have 3 columns for 3D plane fitting.');
    end
    if size(X,1) < 3
         error('Input matrix X must have at least 3 points (rows) to define a plane.');
    end

    % 1. The mean of the samples belongs to the plane
    p = mean(X, 1); % p is a 1x3 row vector (the centroid)

    % 2. Center the data samples by subtracting the mean
    R = bsxfun(@minus, X, p); % R is N x 3

    % 3. Compute the 3x3 covariance matrix
    % Note: R'*R is proportional to the covariance matrix
    C = R' * R;

    % 4. Perform eigenvalue decomposition of the covariance matrix.
    % V_eig columns are the eigenvectors (principal directions).
    % D_eig is a diagonal matrix of eigenvalues (variances).
    % Eigenvectors are orthonormal.
    [V_eig, D_eig] = eig(C);

    % Extract eigenvalues from the diagonal matrix
    eigenvalues = diag(D_eig);

    % 5. The normal vector 'n' corresponds to the eigenvector with the *smallest* eigenvalue.
    [~, min_eval_idx] = min(eigenvalues);
    n = V_eig(:, min_eval_idx); % n is a 3x1 column vector

    % 6. Ensure the normal vector's x-component points towards the origin
    % We check if the x-component of the normal n(1) has the same sign
    % as the x-component of the centroid p(1). If they have the same sign,
    % n(1) is pointing 'away' from the origin relative to p(1), so we flip n.
    % Use a small tolerance for robustness against p(1) or n(1) being near zero.
    if p(1) * n(1) > 1e-10
        n = -n;
    elseif abs(p(1)) < 1e-10 && n(1) < 0 % If centroid is on YZ plane, point towards +x arbitrarily
         n = -n; % Or choose based on another criterion if needed
    end

    
    % 7. The plane basis V consists of the eigenvectors corresponding to the
    %    *two largest* eigenvalues. These vectors are orthogonal to n and span the plane.
    all_indices = 1:3;
    plane_basis_indices = setdiff(all_indices, min_eval_idx);
    V = V_eig(:, plane_basis_indices); % V is 3x2
    %make sure the y vector points in the positive direction
    V(2)=abs(V(2));
        

    % Ensure p is returned as a column vector if desired for consistency,
    % though mean() returns a row vector. Sticking to row vector for p based on mean().
    % If column vector needed: p = p';
    %combine the two matrix into one 3x3;
    T = [n,V];

    %convert to quaternion
    quat = rotm2quat(T);
    % Convert the quaternion to Euler angles
    eul = quat2eul(quat); % Default sequence "ZYX" (output are the three angles in radians)
    %convert to realtive direction coordinates
    xdir=s*sin(eul(2));
    ydir=s*sin(eul(1));
    yaw=eul(3);

end