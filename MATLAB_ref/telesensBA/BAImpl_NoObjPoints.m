function [poses, camParams] = BAImpl_NoObjPoints(objPts, poses, imgPts, f, u0, v0)
    % objPts is 3*numPoints
     
    numPoints = size(objPts, 2);
    % poses is 3*4*numCameras, each pose matrix is of the form [R T]
     
    numCameras = size(poses, 3);
    % wx, wy, wz, tx, ty, tz, f, u0, v0
     
    numCamParams = 9;
     
    numJRows = 2*numPoints*numCameras;
    numJCols = (numCamParams*numCameras);
     
    J = zeros(numJRows, numJCols);
    % fill B and A
     
    B = objPts;
    s = 1;
    for i = 1:numCameras
        R = poses(:,1:3,i);
        T = poses(:, 4, i);
        %[wx wy wz] = rotation_matrix_to_axis_angle(R);
        angles = rotm2eul(R);
        wx = angles(1); wy = angles(2); wz = angles(3);
        A(i,:) = [wx, wy, wz, T(1), T(2), T(3), s*f, u0, v0];
    end
    % imgPts is a numPoints*2 vector
     
    % the "ground truth" for this optimization
     
    u_g = s*imgPts(1,:);
    v_g = s*imgPts(2,:);
    uv_g = [u_g; v_g];
    uv_g = uv_g(:);
     
    err = 10000; err_prev = 10000;
    norm_dp = 100;
    numIter = 0;
    lambda = 0.001;
     
    % Stop the LM iteration of the norm of the change in the parameters
     
    % is less than 1.0e-6 or if numIter > 100
     
    % just using a fixed number of iterations for now
     
    while (numIter < 80) && (norm_dp > 1e-7) && lambda < 1
        numIter = numIter + 1;
        % Fill in the Jacobian matrix
        for j = 1: numCameras,
            wx = A(j,1); wy = A(j,2); wz = A(j,3); 
            tx = A(j,4); ty = A(j,5); tz = A(j,6); 
            f = A(j,7); u0 = A(j,8); v0 = A(j,9);
            for i = 1: numPoints,
                X = B(1,i); Y = B(2,i); Z = B(3,i);
                % x or u coordinates
                J((i-1)*2*numCameras+2*j-1, (j-1)*numCamParams+1: j*numCamParams) = ...
                    [Jx_wx(X,Y,Z,f,tx,tz,u0,wx,wy,wz)
                    Jx_wy(X,Y,Z,f,tx,tz,u0,wx,wy,wz)
                    Jx_wz(X,Y,Z,f,tx,tz,u0,wx,wy,wz)
                    Jx_tx(X,Y,Z,f,tz,wx,wy,wz)
                    0
                    Jx_tz(X,Y,Z,f,tx,tz,u0,wx,wy,wz)
                                         Jx_f(X,Y,Z,tx,tz,wx,wy,wz)
                                         Jx_u0
                                         Jx_v0];
                % y or v coordinates
                J((i-1)*2*numCameras+2*j,(j-1)*numCamParams+1:j*numCamParams) = ...
                    [Jy_wx(X,Y,Z,f,ty,tz,v0,wx,wy,wz)
                    Jy_wy(X,Y,Z,f,ty,tz,v0,wx,wy,wz)
                    Jy_wz(X,Y,Z,f,ty,tz,v0,wx,wy,wz)
                    0
                    Jy_ty(X,Y,Z,f,tz,wx,wy,wz)
                    Jy_tz(X,Y,Z,f,ty,tz,v0,wx,wy,wz)
                                         Jy_f(X,Y,Z,ty,tz,wx,wy,wz)
                                         Jy_u0
                                         Jy_v0];
                
                J((i-1)*2*numCameras+2*j-1, numCamParams*numCameras + 3*(i-1)+1: numCamParams*numCameras + 3*i) = ...
                    [ Jx_X(X,Y,Z,f,tx,tz,u0,wx,wy,wz)
                    Jx_Y(X,Y,Z,f,tx,tz,u0,wx,wy,wz)
                    Jx_Z(X,Y,Z,f,tx,tz,u0,wx,wy,wz)];
                
                
                J((i-1)*2*numCameras+2*j, numCamParams*numCameras + 3*(i-1)+1: numCamParams*numCameras + 3*i) = ...
                    [ Jy_X(X,Y,Z,f,ty,tz,v0,wx,wy,wz)
                    Jy_Y(X,Y,Z,f,ty,tz,v0,wx,wy,wz)
                    Jy_Z(X,Y,Z,f,ty,tz,v0,wx,wy,wz)];
            end
        end
     
        uv_r = computeReprojectedPts(A, B, numCameras,numPoints);
        d = [uv_r - uv_g];
        H = J'*J;
        % Apply the damping factor to the Hessian matrix
        %H_lm=H+(lambda*eye(numJCols,numJCols));
        H_lm=H+(lambda*diag(diag(H)));
        dp=-inv(H_lm)*(J'*d(:));
        norm_dp = norm(dp);
        % Apply updates and compute new error. The update is contingent on
        % actually reducing the reprojection error
        A_temp = zeros(numCameras, numCamParams);
        for i = 1: numCameras,
            for j = 1: numCamParams,
                A_temp(i,j) = A(i,j) + dp((i-1)*numCamParams + j);
            end
        end
        uv_r = computeReprojectedPts(A_temp, B, numCameras,numPoints);
        d = [uv_r - uv_g];
        err = d'*d;
        
        if (err < err_prev)
            lambda = lambda/2;
            err_prev = err;
            A = A_temp;
        else
            lambda = lambda*2;
        end
    end % end of Levenberg Marquardt
     
    for i = 1:numCameras
        %R = axis_angle_to_rotation_matrix(A(i,1), A(i,2), A(i,3));
        R = eul2rotm(A(i,1), A(i,2), A(i,3));
        T = [A(i,4) A(i,5) A(i,6)];
        poses(:, 1:3,i) = R;
        poses(:, 4, i) = T';    
        camParams(i, 1:3) = [A(i,7) A(i,8) A(i,9)];
    end
    objPts = B;
end