%T2R Return rotational submatrix of a homogeneous transformation
%
% R = T2R(T) is the orthonormal rotation matrix component of homogeneous 
% transformation matrix T:
%
% Notes::
% - functions for T in SE(2) or SE(3)
%   - If T is 4x4, then R is 3x3.
%   - If T is 3x3, then R is  2x2.
% - the validity of rotational part is not checked
%
% See also R2T, TR2RT, RT2TR.


% Copyright (C) 1993-2011, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for Matlab (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.

function R = t2r(T)

    if numcols(T) ~= numrows(T)
        error('T must be square');
    end

    n = numcols(T);     % works for SE(2) or SE(3)
	R = T(1:n-1,1:n-1);
