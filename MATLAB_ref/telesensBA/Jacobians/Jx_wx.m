function Jx_wx = Jx_wx(X,Y,Z,f,tx,tz,u0,wx,wy,wz)
%JX_WX
%    JX_WX = JX_WX(X,Y,Z,F,TX,TZ,U0,WX,WY,WZ)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    20-Jul-2022 14:06:11

t2 = wx.^2;
t3 = wy.^2;
t4 = wz.^2;
t5 = t2+t3;
t6 = t3+t4;
t7 = t4+t5;
t8 = 1.0./t7;
t10 = sqrt(t7);
t9 = t8.^2;
t11 = 1.0./t10;
t13 = cos(t10);
t14 = sin(t10);
t12 = t11.^3;
t15 = t13-1.0;
t16 = t8.*t13.*wx.*wy;
t17 = t11.*t14;
t18 = t2.*t8.*t13;
t19 = t17.*wx;
t20 = t17.*wy;
t21 = t8.*t15.*wz;
t22 = -t16;
t23 = t12.*t14.*wx.*wy;
t24 = t8.*t15.*wx.*2.0;
t25 = t2.*t12.*t14;
t32 = t9.*t15.*wx.*wy.*wz.*2.0;
t35 = t2.*t9.*t15.*wz.*2.0;
t36 = t5.*t8.*t15;
t37 = t5.*t12.*t14.*wx;
t39 = t5.*t9.*t15.*wx.*2.0;
t26 = t21.*wx;
t27 = t21.*wy;
t28 = t23.*wz;
t29 = -t24;
t30 = -t21;
t31 = t25.*wz;
t33 = -t25;
t38 = t36+1.0;
t34 = -t27;
t40 = Z.*t38;
t41 = t20+t26;
t46 = t29+t37+t39;
t47 = t17+t18+t28+t32+t33;
t48 = t22+t23+t30+t31+t35;
t42 = X.*t41;
t43 = t19+t34;
t44 = -t42;
t45 = Y.*t43;
t49 = t40+t44+t45+tz;
Jx_wx = (-Z.*(t46.*u0-f.*(t16-t23+t30+t31+t35))+Y.*(t47.*u0+f.*(t25.*wy-t8.*t15.*wy+t2.*t9.*t15.*wy.*2.0-t8.*t13.*wx.*wz+t12.*t14.*wx.*wz))+X.*(t48.*u0-f.*(t6.*t9.*t15.*wx.*2.0+t6.*t12.*t14.*wx)))./t49-1.0./t49.^2.*(X.*t48+Y.*t47-Z.*t46).*(Y.*(t43.*u0-f.*(t17.*wz+t8.*t15.*wx.*wy))+f.*tx-X.*(t41.*u0-f.*(t6.*t8.*t15+1.0))+tz.*u0+Z.*(t38.*u0+f.*(t20-t26)));
