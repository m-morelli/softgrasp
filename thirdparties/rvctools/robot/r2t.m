%R2T Convert rotation matrix to a homogeneous transform
%
% T = R2T(R) is a homogeneous transform equivalent to an orthonormal 
% rotation matrix R with a zero translational component.
%
% Notes::
% - functions for T in SE(2) or SE(3)
%  - if R is 2x2 then T is 3x3, or
%  - if R is 3x3 then T is 4x4.
% - translational component is zero
%
% See also T2R.


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

function T = r2t(R)

    n = numrows(R);

	T = [R zeros(n,1); zeros(1,n) 1];
