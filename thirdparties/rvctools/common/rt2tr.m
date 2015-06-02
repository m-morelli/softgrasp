%RT2TR Convert rotation and translation to homogeneous transform
%
% TR = RT2TR(R, t) is a homogeneous transformation matrix formed from an 
% orthonormal rotation matrix R and a translation vector t.
%
% Notes::
% - functions for R in SO(2) or SO(3)
%  - If R is 2x2 and t is 2x1, then TR is 3x3
%  - If R is 3x3 and t is 3x1, then TR is 4x4
% - the validity of R is not checked
%
% See also T2R, R2T, TR2RT.

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

function T = rt2tr(R, t)
    if numcols(R) ~= numrows(R)
        error('R must be square');
    end
    if numrows(R) ~= numrows(t)
        error('R and t must have the same number of rows');
    end

    T = [R t; zeros(1,numcols(R)) 1];

