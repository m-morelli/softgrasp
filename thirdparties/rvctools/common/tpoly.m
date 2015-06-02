%TPOLY Generate scalar polynomial trajectory
%
% [S,SD,SDD] = TPOLY(S0, SF, N) is a trajectory of a scalar that varies 
% smoothly from S0 to SF in N steps using a quintic (5th order) polynomial.
% Velocity and acceleration can be optionally returned as SD and SDD.  The 
% trajectory S, SD and SDD are N-vectors.
%
% [S,SD,SDD] = TPOLY(S0, SF, T) as above but specifies the trajectory in 
% terms of the length of the time vector T.

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

% [S,SD,SDD] = TPOLY(S0, SF, N, SD0, SDF) as above but specifies initial 
% and final joint velocity for the trajectory.
%
% [S,SD,SDD] = TPOLY(S0, SF, T, SD0, SDF) as above but specifies initial 
% and final joint velocity for the trajectory and time vector T.
%
% Notes::
% - In all cases if no output arguments are specified S, SD, and SDD are plotted 
%   against time.
%
% See also LSPB, JTRAJ.

function [s,sd,sdd] = tpoly(q0, qf, t, qd0, qdf)

    if isscalar(t)
        t = [0:(t-1)]';
    else
        t = t(:);
    end
    if nargin < 4
        qd0 = 0;
    end
    if nargin < 5
        qdf = 0;
    end
    
    tf = max(t);
    
    % solve for the polynomial coefficients using least squares
    X = [
        0           0           0         0       0   1
        tf^5        tf^4        tf^3      tf^2    tf  1
        0           0           0         0       1   0
        5*tf^4      4*tf^3      3*tf^2    2*tf    1   0
        0           0           0         2       0   0
        20*tf^3     12*tf^2     6*tf      2       0   0
    ];
    coeffs = (X \ [q0 qf qd0 qdf 0 0]')';

    % coefficients of derivatives 
    coeffs_d = [coeffs(1:5)] .* [5:-1:1];
    coeffs_dd = [coeffs_d(1:4)] .* [4:-1:1];

    % evaluate the polynomials
    p = polyval(coeffs, t);
    pd = polyval(coeffs_d, t);
    pdd = polyval(coeffs_dd, t);

    switch nargout
        case 0
            subplot(311)
            plot(t, p); grid; ylabel('s');
            subplot(312)
            plot(t, pd); grid; ylabel('sd');
            subplot(313)
            plot(t, pdd); grid;  ylabel('sdd');
            xlabel('time')
            shg
        case 1
            s = p;
        case 2
            s = p;
            sd = pd
        case 3
            s = p;
            sd = pd;
            sdd = pdd;
    end
