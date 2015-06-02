%TR2ANGVEC Convert rotation matrix to angle-vector form
%
% [THETA,V] = TR2ANGVEC(R) is a rotation of THETA about the vector V 
% equivalent to the orthonormal rotation matrix R.
%
% [THETA,V] = TR2ANGVEC(T) is a rotation of THETA about the vector V 
% equivalent to the rotational component of the homogeneous transform T.
%
% Notes::
% - If no output arguments are specified the result is displayed.
%
% See also ANGVEC2R, ANGVEC2TR.


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

function [theta, v] = tr2angvec(R)

    if ~isrot(R)
        R = t2r(R);
    end
    
	qs = sqrt(trace(R)+1)/2.0;
    qs = min(qs, 1);
        
	kx = R(3,2) - R(2,3);	% Oz - Ay
	ky = R(1,3) - R(3,1);	% Ax - Nz
	kz = R(2,1) - R(1,2);	% Ny - Ox

	if (R(1,1) >= R(2,2)) && (R(1,1) >= R(3,3)) 
		kx1 = R(1,1) - R(2,2) - R(3,3) + 1;	% Nx - Oy - Az + 1
		ky1 = R(2,1) + R(1,2);			% Ny + Ox
		kz1 = R(3,1) + R(1,3);			% Nz + Ax
		add = (kx >= 0);
	elseif (R(2,2) >= R(3,3))
		kx1 = R(2,1) + R(1,2);			% Ny + Ox
		ky1 = R(2,2) - R(1,1) - R(3,3) + 1;	% Oy - Nx - Az + 1
		kz1 = R(3,2) + R(2,3);			% Oz + Ay
		add = (ky >= 0);
	else
		kx1 = R(3,1) + R(1,3);			% Nz + Ax
		ky1 = R(3,2) + R(2,3);			% Oz + Ay
		kz1 = R(3,3) - R(1,1) - R(2,2) + 1;	% Az - Nx - Oy + 1
		add = (kz >= 0);
	end

	if add
		kx = kx + kx1;
		ky = ky + ky1;
		kz = kz + kz1;
	else
		kx = kx - kx1;
		ky = ky - ky1;
		kz = kz - kz1;
	end
	v = unit([kx ky kz]);
    theta = 2*acos(qs);
    if theta > pi/2
        theta = pi - theta;
        v = -v;
    end

    if nargout == 0
        fprintf('Rotation: %f rad x [%f %f %f]\n', theta, v(1), v(2), v(3));
    end
