function uv_r = computeReprojectedPts(A, B, numCameras, numPoints)
    u_= []; v_ = [];
    for j = 1: numCameras,
        wx = A(j,1); wy = A(j,2); wz = A(j,3);
        tx = A(j,4); ty = A(j,5); tz = A(j,6);
        f = A(j,7); u0 = A(j,8); v0 = A(j,9);
        
        [u v] = proj3dto2d(B, wx, wy, wz, tx, ty, tz,f, u0, v0);
        u_ = [u_ u]; v_ = [v_ v];
    end
    u_r = reshape(u_, numPoints, numCameras)';
    u_r = u_r(:)';
     
    v_r = reshape(v_, numPoints, numCameras)';
    v_r = v_r(:)';
    % Interleave u and vs
     
    uv_r = [u_r; v_r];
    uv_r = uv_r(:);
end