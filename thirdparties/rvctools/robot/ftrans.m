%WTRANS Transform a wrench between coordinate frames
%
% WT = WTRANS(T, W) is a wrench in the frame T corresponding to the world
% frame wrench W.  
%
% The wrenches W and WT are 6-vectors of the form [Fx Fy Fz Mx My Mz].
%
% See also TR2DELTA.


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

function Wt = wtrans(T, W)

    f = W(1:3); m = W(4:6);
    k = cross(f, transl(T) ) + m;

    mt = rot(T)' * k;
    ft = rot(T)' * F(1:3);

    Wt = [ft; mt];
