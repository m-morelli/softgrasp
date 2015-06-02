%EUL2R Convert Euler angles to homogeneous transformation
%
% R = EUL2R(PHI, THETA, PSI) returns an orthonornal rotation matrix equivalent to
% the specified Euler angles.  These correspond to rotations about the Z, Y, Z 
% axes respectively. If PHI, THETA, PSI are column vectors then they are assumed to
% represent a trajectory and R is a three dimensional matrix, where the last 
% index corresponds to rows of PHI, THETA, PSI.
%
% R = EUL2R(EUL) as above but the Euler angles are taken from
% consecutive columns of the passed matrix EUL = [PHI THETA PSI].
%
% Note::
% - the vectors PHI, THETA, PSI must be of the same length
%
% See also EUL2TR, RPY2TR, TR2EUL.


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

function r = eul2r(phi, theta, psi)
    if (nargin == 1)
        if numcols(phi) ~= 3
            error('bad arguments')
        end
        theta = phi(:,2);
        psi = phi(:,3);
        phi = phi(:,1);
    end

    if numrows(phi) == 1
        r = rotz(phi) * roty(theta) * rotz(psi);
    else
        for i=1:numrows(phi)
            r(:,:,1) = rotz(phi(i)) * roty(theta(i)) * rotz(psi(i));
        end

                
    end
