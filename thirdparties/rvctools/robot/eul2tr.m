%EUL2TR Convert Euler angles to homogeneous transform
%
% T = EUL2TR(PHI, THETA, PSI) returns a homogeneous tranformation equivalent to
% the specified Euler angles.  These correspond to rotations about the Z, Y, Z 
% axes respectively. If PHI, THETA, PSI are column vectors then they are assumed to
% represent a trajectory and T is a three dimensional matrix, where the last 
% index corresponds to rows of PHI, THETA, PSI.
%
% T = EUL2TR(EUL) as above but the Euler angles are taken from
% consecutive columns of the passed matrix EUL = [PHI THETA PSI].
%
% Note::
% - the vectors PHI, THETA, PSI must be of the same length
% - the translational part is zero.
%
% See also EUL2R, RPY2TR, TR2EUL.


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

function T = eul2tr(phi, theta, psi)
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
        T = r2t(r);
    else
        T = [];
        for i=1:numrows(phi)
            r = rotz(phi(i)) * roty(theta(i)) * rotz(psi(i));
            T(:,:,i) = r2t(r);
        end
    end
