function R = axis_angle_to_rotation_matrix(wx, wy, wz)
    theta=sqrt(wx^2+wy^2+wz^2);
    omega=  [0 -wz wy;
             wz 0 -wx;
            -wy wx 0;];
    R = eye(3) + (sin(theta)/theta)*omega + ((1-cos(theta))/theta^2)*(omega*omega);
end