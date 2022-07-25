syms u0 v0 f real
syms tx ty tz wx wy wz real
syms X Y Z real

% Ihe intrinsic parameter matrix
K=[f 0 u0;
    0 f v0;
    0 0 1];

% Expression for the rotation matrix based on the Rodrigues formula
theta=sqrt(wx^2+wy^2+wz^2);
omega=  [0 -wz wy;
         wz 0 -wx;
        -wy wx 0;];
R = eye(3) + (sin(theta)/theta)*omega + ((1-cos(theta))/theta^2)*(omega*omega);
% Expression for the translation vector
t=[tx;ty;tz];
% perspective projection of the model point (X,Y,Z)
uvs=K*[R t]*[X; Y; Z; 1];
u=uvs(1)/uvs(3);
v=uvs(2)/uvs(3);
% calculate the geometric distance in x and y direction
% u,v = the x and y positions of the projection of the corresponding model point
dx=u;
dy=v;
% Evaluate the symbolic expression of the Jacobian w.r.t. the estimated parameters
% Jx=jacobian(dx,[au,av,u0,v0,wx wy wz tx ty tz]);
% Jy=jacobian(dy,[au,av,u0,v0,wx wy wz tx ty tz]);
Jx_f = jacobian(dx,f);             
Jx_u0 = jacobian(dx,[u0]);        
Jx_v0 = jacobian(dx,[v0]);    
Jx_wx = jacobian(dx,[wx]);
Jx_wy = jacobian(dx,[wy]);
Jx_wz = jacobian(dx,[wz]);
Jx_tx = jacobian(dx,[tx]);
Jx_ty = jacobian(dx,[ty]);
Jx_tz = jacobian(dx,[tz]);
Jx_X = jacobian(dx,[X]);
Jx_Y = jacobian(dx,[Y]);
Jx_Z = jacobian(dx,[Z]);

Jy_f = jacobian(dy,f);
Jy_u0 = jacobian(dy,[u0]);
Jy_v0 = jacobian(dy,[v0]);
Jy_wx = jacobian(dy,[wx]);
Jy_wy = jacobian(dy,[wy]);
Jy_wz = jacobian(dy,[wz]);
Jy_tx = jacobian(dy,[tx]);
Jy_ty = jacobian(dy,[ty]);
Jy_tz = jacobian(dy,[tz]);
Jy_X = jacobian(dy,[X]);
Jy_Y = jacobian(dy,[Y]);
Jy_Z = jacobian(dy,[Z]);

% Convert to matlab functions that are stored as m files in the Jacobians directory. These functions
% can be called directly from other scripts.  
Jx_u0_func = matlabFunction(Jx_u0, 'File', './Jacobians/Jx_u0');
Jx_v0_func = matlabFunction(Jx_v0, 'File', './Jacobians/Jx_v0');
Jx_wx_func = matlabFunction(Jx_wx, 'File', './Jacobians/Jx_wx');
Jx_wy_func = matlabFunction(Jx_wy, 'File', './Jacobians/Jx_wy')
Jx_wz_func = matlabFunction(Jx_wz, 'File', './Jacobians/Jx_wz')
Jx_tx_func = matlabFunction(Jx_tx, 'File', './Jacobians/Jx_tx')
Jx_ty_func = matlabFunction(Jx_ty, 'File', './Jacobians/Jx_ty')
Jx_tz_func = matlabFunction(Jx_tz, 'File', './Jacobians/Jx_tz')
Jx_f_func = matlabFunction(Jx_f, 'File', './Jacobians/Jx_f')
Jx_X_func = matlabFunction(Jx_X, 'File', './Jacobians/Jx_X')
Jx_Y_func = matlabFunction(Jx_Y, 'File', './Jacobians/Jx_Y')
Jx_Z_func = matlabFunction(Jx_Z, 'File', './Jacobians/Jx_Z')

Jy_u0_func = matlabFunction(Jy_u0, 'File', './Jacobians/Jy_u0');
Jy_v0_func = matlabFunction(Jy_v0, 'File', './Jacobians/Jy_v0');
Jy_wx_func = matlabFunction(Jy_wx, 'File', './Jacobians/Jy_wx');
Jy_wy_func = matlabFunction(Jy_wy, 'File', './Jacobians/Jy_wy')
Jy_wz_func = matlabFunction(Jy_wz, 'File', './Jacobians/Jy_wz')
Jy_tx_func = matlabFunction(Jy_tx, 'File', './Jacobians/Jy_tx')
Jy_ty_func = matlabFunction(Jy_ty, 'File', './Jacobians/Jy_ty')
Jy_tz_func = matlabFunction(Jy_tz, 'File', './Jacobians/Jy_tz')
Jy_f_func = matlabFunction(Jy_f, 'File', './Jacobians/Jy_f')
Jy_X_func = matlabFunction(Jy_X, 'File', './Jacobians/Jy_X')
Jy_Y_func = matlabFunction(Jy_Y, 'File', './Jacobians/Jy_Y')
Jy_Z_func = matlabFunction(Jy_Z, 'File', './Jacobians/Jy_Z')