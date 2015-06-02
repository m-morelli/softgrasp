%SerialLink.INERTIA Manipulator inertia matrix
%
% I = R.inertia(Q) is the NxN symmetric joint inertia matrix which relates 
% joint torque to joint acceleration for the robot at joint configuration Q.
% The diagonal elements I(j,j) are the inertia seen by joint actuator j.
% The off-diagonal elements are coupling inertias that relate acceleration
% on joint i to force/torque on joint j.
%
% See also SerialLink.RNE, SerialLink.CINERTIA, SerialLink.ITORQUE.



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
%
% http://www.petercorke.com

function M = inertia(robot, q)
	n = robot.n;

	if numel(q) == robot.n,
		q = q(:)';
	end

	M = zeros(n,n,0);
	for Q = q',
		m = rne(robot, ones(n,1)*Q', zeros(n,n), eye(n), [0;0;0]);
		M = cat(3, M, m);
	end
