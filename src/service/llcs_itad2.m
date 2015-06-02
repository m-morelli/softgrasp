function out = llcs_itad2(h)
%AD  Performs the transpose of the inverse of adjoint transform 
%
%	A = AD^{-T}(x)
%
if any(size(h)==[4,4])
    Raux=(h((1:3),(1:3)));
    daux=h((1:3),(4));
    out=[Raux , zeros(3,3) ;
        -[skew(daux)]'*Raux , Raux];
else
    error('The argument of "ad2_MT" function is not correct')
end
