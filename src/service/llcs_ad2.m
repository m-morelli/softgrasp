function out = llcs_ad2(h)
%AD  Performs the adjoint transform
%
%	A = AD(x)
%
if any(size(h)==[4,4])
    Raux=(h((1:3),(1:3)));
    daux=h((1:3),(4));
    out=[Raux , skew(daux)*Raux ;
        zeros(3,3) , Raux];
else
    error('The argument of "ad2" function is not correct')
end
