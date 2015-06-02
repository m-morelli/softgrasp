%MTRAJ Multi-axis trajectory between two points
%
% [Q,QD,QDD] = MTRAJ(TFUNC, Q0, QF, N) is multi-axis trajectory Q varying
% from state Q0 to QF according to the scalar trajectory function TFUNC in
% N steps. Joint velocity and acceleration can be optionally returned as 
% QD and QDD respectively. The trajectory Q, QD and QDD are MxN matrices, 
% with one row per time step, and one column per axis.
%
% The shape of the trajectory is given by the scalar trajectory function TFUNC
%   [S,SD,SDD] = TFUNC(S0, SF, N);
% and possible values of TFUNC include @lspb for a trapezoidal trajectory, or
% @tpoly for a polynomial trajectory.
%
% [Q,QD,QDD] = MTRAJ(TFUNC, Q0, QF, T) as above but specifies the trajectory 
% length in terms of the length of the time vector T.
%
% Notes::
% - when TFUNC is @tpoly the result is similar to JTRAJ except that no
%   initial velocities can be specified. JTRAJ is computationally a little
%   more efficient.
%
% See also JTRAJ, MSTRAJ, LSPB, TPOLY.

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

function [s,Sd,Sdd] = mtraj(tfunc, q0, qf, N)

    if ~isscalar(N)
        N = length(N);
    end
    if numcols(q0) ~= numcols(qf)
        error('must be same number of columns in q0 and qf')
    end

    s = zeros(N, numcols(q0));
    sd = zeros(N, numcols(q0));
    sdd = zeros(N, numcols(q0));

    for i=1:numcols(q0)
        % for each axis
        [s(:,i),sd(:,i),sdd(:,i)] = tfunc(q0(i), qf(i), N);
    end

    if nargout > 1
        Sd = sd;
    end
    if nargout > 2
        Sdd = sdd;
    end
