%CTRAJ Cartesian trajectory between two points
%
% TC = CTRAJ(T0, T1, N) is a Cartesian trajectory from pose T0 to T1
% with N points that follow a trapezoidal velocity profile along the path.
% The Cartesian trajectory is a 4x4xN matrix, with the last subscript being the
% point index.
%
% TC = CTRAJ(T0, T1, S) as above but the elements of S specify the fractional 
% distance  along the path, and these values are in the range [0 1].
% The Cartesian trajectory is a 4x4xN matrix, with transform T(:,:,i) 
% corresponding to S(i).
%
% See also MSTRAJ, TRINTERP, Quaternion.interp, TRANSL.


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

function traj = ctraj(T0, T1, t)

    if ~ishomog(T0) || ~ishomog(T1)
        error('arguments must be homogeneous transformations');
    end
    
    % distance along path is a smooth function of time
    if isscalar(t)
        s = lspb(0, 1, t);
    else
        s = t(:);
    end

    traj = [];

    for S=s'
        traj = cat(3, traj, trinterp(T0, T1, S));
    end

    
