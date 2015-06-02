%RPY2TR Roll-pitch-yaw angles to homogeneous transform
%
% T = RPY2TR(RPY) is an orthonormal rotation matrix equivalent to the specified 
% roll, pitch, yaw angles which correspond to rotations about the X, Y, Z axes 
% respectively. If RPY has multiple rows they are assumed to represent a 
% trajectory and R is a three dimensional matrix, where the last index  
% corresponds to the rows of RPY.
%
% T = RPY2TR(ROLL, PITCH, YAW) as above but the roll-pitch-yaw angles are passed
% as separate arguments.
%
% If ROLL, PITCH and YAW are column vectors then they are assumed to represent a 
% trajectory and R is a three dimensional matrix, where the last index 
% corresponds to the rows of ROLL, PITCH, YAW.
%
% Note::
% - in previous releases (<8) the angles corresponded to rotations about ZYX.
% - many texts (Paul, Spong) use the rotation order ZYX.
%
% See also TR2RPY, EUL2TR.


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

function T = rpy2tr(roll, pitch, yaw)
	if (nargin == 1)
		if numcols(roll) ~= 3
			error('bad arguments')
		end
		pitch = roll(:,2);
		yaw = roll(:,3);
		roll = roll(:,1);
	end

	if numrows(roll) == 1
		r = rotx(roll) * roty(pitch) * rotz(yaw);
		T = r2t(r);
	else
		for i=1:numrows(roll)
			r = rotx(roll(i)) * roty(pitch(i)) * rotz(yaw(i));
			T(:,:,i) = r2t(r);
		end
	end
