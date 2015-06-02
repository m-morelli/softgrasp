function out = llcs_homogeneous(R,d)
%
%
%	It build the homogeneous matrix starting from rotation matrix and 
%   traslation
%
if any(size(d)==[1,3])
    d=d';
    if any(size(R)==[3,3])
        out=[R,d; zeros(1,3) ,1];
    else
        error('The arguments of "homogeneous" function are not correct')
    end
    
elseif any(size(d)==[3,1])
    if any(size(R)==[3,3])
        out=[R,d; zeros(1,3) ,1];
    else
        error('The arguments of "homogeneous" function are not correct')
    end
else
    error('The arguments of "homogeneous" function are not correct')
end