clear all;
close all;
clc;

dwx=0.1;
wx = 1;
wy = 2;
wz = 3;
tx = 0.1;
ty = 0.2;
tz = 0.3;
f = 0.5;
u0 = 0;
v0 = 0;

X = 1;
Y = 3;
Z = 5;

%Jx_wx_arr = zeros(1,100);
u_prev = u0;
idx = 1;
for wx_val = wx - 5:dwx:wx + 5,
    %[u v] = proj3dto2d(pts3d, wx_val, wy, wz, tx, ty, tz, f, u0, v0);
    [u v] = proj3dto2d([X; Y; Z; 1], wx_val, wy, wz, tx, ty, tz, f, u0, v0);
    % compute the value of the derivative manually
    
    du_dwx = (u - u_prev)/dwx;
    u_prev = u;

    % compute the value from the jacobian
    Jx_wx_arr(idx) = Jx_wx(X, Y, Z, f, tx, tz, u0, wx_val, wy, wz);
    du_dwx_arr(idx) = du_dwx;
    R = axis_angle_to_rotation_matrix(wx_val, wy, wz);
    u_arr(idx) = u;
    wx_arr(idx) = wx_val;
    idx = idx+1;
end

figure;
hold on;
plot(wx_arr, Jx_wx_arr);
plot(wx_arr, du_dwx_arr);
legend('Jx\_wx\_arr', 'du\_dwx\_arr');
