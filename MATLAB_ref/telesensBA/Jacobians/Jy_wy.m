function Jy_wy = Jy_wy(X,Y,Z,f,ty,tz,v0,wx,wy,wz)
%JY_WY
%    JY_WY = JY_WY(X,Y,Z,F,TY,TZ,V0,WX,WY,WZ)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    20-Jul-2022 14:06:13

t2 = wx.^2;
t3 = wy.^2;
t4 = wz.^2;
t5 = t2+t3;
t6 = t2+t4;
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
t18 = t3.*t8.*t13;
t19 = t17.*wx;
t20 = t17.*wy;
t21 = t8.*t15.*wz;
t22 = t12.*t14.*wx.*wy;
t23 = -t17;
t24 = t8.*t15.*wy.*2.0;
t25 = -t18;
t26 = t3.*t12.*t14;
t34 = t9.*t15.*wx.*wy.*wz.*2.0;
t36 = t3.*t9.*t15.*wz.*2.0;
t37 = t5.*t8.*t15;
t38 = t5.*t12.*t14.*wy;
t40 = t5.*t9.*t15.*wy.*2.0;
t27 = t21.*wx;
t28 = t21.*wy;
t29 = t22.*wz;
t30 = -t24;
t31 = -t21;
t32 = -t22;
t33 = t26.*wz;
t39 = t37+1.0;
t35 = -t28;
t41 = Z.*t39;
t42 = t20+t27;
t47 = t30+t38+t40;
t48 = t23+t25+t26+t29+t34;
t49 = t16+t31+t32+t33+t36;
t43 = X.*t42;
t44 = t19+t35;
t45 = -t43;
t46 = Y.*t44;
t50 = t41+t45+t46+tz;
Jy_wy = (-Z.*(t47.*v0-f.*(-t16+t22+t31+t33+t36))+X.*(t48.*v0+f.*(t26.*wx-t8.*t15.*wx+t3.*t9.*t15.*wx.*2.0+t8.*t13.*wy.*wz-t12.*t14.*wy.*wz))+Y.*(t49.*v0-f.*(t6.*t9.*t15.*wy.*2.0+t6.*t12.*t14.*wy)))./t50-1.0./t50.^2.*(X.*t48+Y.*t49-Z.*t47).*(-X.*(t42.*v0-f.*(t17.*wz-t8.*t15.*wx.*wy))+f.*ty+Y.*(t44.*v0+f.*(t6.*t8.*t15+1.0))+tz.*v0-Z.*(f.*(t19+t28)-t39.*v0));
