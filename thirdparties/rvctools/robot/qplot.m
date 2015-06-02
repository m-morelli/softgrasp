%QPLOT Plot joint angles
%
% QPLOT(Q) is a convenience function to plot joint angle trajectories for a
% 6-axis robot.  Q is Nx6, and the first three joints are shown as solid lines,
% the last three joints are shown as dashed lines.  A legend is also
% displayed.
%
% QPLOT(T, Q) displays the joint angle trajectory versus time T. 
%
% See also JTRAJ.

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

function qplot(t, q)
    if nargin < 2
        q = t;
        t = [1:numrows(q)]';
    end
    clf
    hold on
    plot(t, q(:,1:3))
    plot(t, q(:,4:6), '--')
    grid on
    xlabel('time')
    ylabel('q')
    legend('1', '2', '3', '4', '5', '6');
    hold off

    xlim([t(1), t(end)]);
