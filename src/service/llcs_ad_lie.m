function [ad_lie] = llcs_ad_lie(xi)
%
% Dato in ingresso un twist xi1 = [v, omega]'
% fornisce in uscita la forma ad(xi1) tale che
% ad(xi1)*xi2 = LieBracket[xi1, xi2] =
% =hat(x1)*hat(x2)-hat(x2)*hat(x1)
%

if all(size(xi)==[4,4])
    omega = ext_skewcoords(xi(1:3,1:3)) ;
    v = xi(1:3,4) ;
    ad_lie = [skew(omega), skew(v);
    zeros(3), skew(omega)];
elseif all(size(xi)==[1,6])
    xi=xi' ;
    v =  xi(1:3,1) ;
    omega =xi(4:6,1) ;
    ad_lie = [skew(omega), skew(v);
    zeros(3), skew(omega)];
elseif all(size(xi)==[6,1])
    v =  xi(1:3,1) ;
    omega =xi(4:6,1) ;
    ad_lie = [skew(omega), skew(v);
        zeros(3), skew(omega)];
else
    disp('not a twist')
end
