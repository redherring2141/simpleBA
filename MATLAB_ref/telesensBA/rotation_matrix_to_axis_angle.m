function [wx wy wz] = rotation_matrix_to_axis_angle(R)
    wx = atan2(R(3,2), R(3,3));
    wy = atan2(-R(3,1), sqrt(R(3,2)^2+R(3,3)^2));
    wz = atan2(R(2,1), R(1,1));
end

%function [wx wy wz] = rotation_matrix_to_axis_angle(R)
%    wx = atan2(R(2,3), R(3,3));
%    wy = atan2(-R(1,3), sqrt(R(2,3)^2+R(3,3)^2));
%    wz = atan2(R(1,2), R(1,1));
%end