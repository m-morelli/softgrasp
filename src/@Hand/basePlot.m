%Hand.BASEPLOT. Perform 3D Graphical Animation of a Hand Model for a Given
%Joint-Set
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Matteo Morelli, I.R.C. "E. Piaggio", University of Pisa
%
%   March 2013

% Copyright (C) 2013 Interdepartmental Research Center "E. Piaggio", University of Pisa
%
% This file is part of THE Robotic Grasping Toolbox for use with MATLAB(R).
%
% THE Robotic Grasping Toolbox for use with MATLAB(R) is free software: you can redistribute it and/or modify it under
% the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the
% License, or (at your option) any later version.
%
% THE Robotic Grasping Toolbox for use with MATLAB(R) is distributed in the hope that it will be useful, but WITHOUT ANY
% WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
% General Public License for more details.
%
% You should have received a copy of the GNU General Public License along with THE Robotic Grasping Toolbox for use with
% MATLAB(R).  If not, see <http://www.gnu.org/licenses/>.

function conf = basePlot(hnd, q, PlotOpt)

    % q must be a row vector. It is assumed as a row vector
    % No err check performed.

    % ret
    conf = [];

    % DoFs, finger numbers etc.
    fgrNo = hnd.nLimbs;
    fJtNo = hnd.dofsPerLimb;

    % actual base plot
    q0 = 1;
    for i = 1:fgrNo,
        ithLimb = Limb(hnd.limbs(i));
        %ithLimb.base = hnd.palm*ithLimb.base; %ithLimb = hnd.limbs(i);
        qf = q0 - 1 + fJtNo(i);
        ithLimb.plot(q(1,q0:qf), PlotOpt); % plot a new limb
        q0 = qf + 1;
    end

end