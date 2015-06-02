 function g = llcs_twistexp2(xi, theta)
%
%
%Calcolo dell'esponenziale e^{\hat{xi}*theta}

if (isrow(xi))
    xi=xi';
end
v = xi(1:3);
w = xi(4:6);
w_hat = ext_rot(ext_twist(xi));
%
if  w_hat == zeros(3,3)
    g_11 = eye(3);
    g_12 = v*theta;
    g_21 = zeros(1,3);
    g_22 = 1 ;
else
    norm_omega = sqrt(sum(w.*w));
    norm_omega2 = (sum(w.*w));
    I = eye(3);
    e_om_th = I+(w_hat./norm_omega)*sin(norm_omega*theta)+((w_hat^2)./norm_omega2)*(1-cos(norm_omega*theta));
    %
    g_11 = e_om_th;
    g_12 = (I-e_om_th)*(cross(w,v))+(w*w')*v*theta;
    g_21 = zeros(1,3);
    g_22 = 1 ;
end
g = [g_11, g_12;g_21, g_22];

%
