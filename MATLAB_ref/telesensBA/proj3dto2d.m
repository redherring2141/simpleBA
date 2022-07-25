function [u, v] = proj3dto2d(pts3d, wx, wy, wz, tx, ty, tz, f, u0, v0)
    % Intrinsic params matrix
     
    K = [   f 0 u0;
        0 f v0;
        0 0 1];
    % Expression for the rotation matrix based on the Rodrigues formula
     
    %R = axis_angle_to_rotation_matrix(wx, wy, wz);
    R = eul2rotm(wx, wy, wz);
     
    % Expression for the translation vector
     
    t=[tx;ty;tz];
     
    % perspective projection of the model point (X,Y)
     
    uvs=K*[R t]*pts3d;
    u=uvs(1,:)./uvs(3,:);
    v=uvs(2,:)./uvs(3,:);
end