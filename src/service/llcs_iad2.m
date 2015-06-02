function out = llcs_iad2(h)
%AD  Performs the inverse of the adjoint transform
%
%	A = IAD(x)
%
if any(size(h)==[4,4])
    Raux=(h((1:3),(1:3)));
    daux=h((1:3),(4));
    out=[Raux' , -Raux'*skew(daux) ;...
        zeros(3,3) , Raux'];
else
    error('The argument of "iad2" function is not correct')
end
