%Limb.PLOT. Perform 3D Graphical Animation of a Limb Model for a Given Joint-Trajectory
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Matteo Morelli, I.R.C. "E. Piaggio", University of Pisa
%   Nicola Greco, I.R.C. "E. Piaggio", University of Pisa
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

function retval = plot(limb, q, PlotOpt_)

    % number of rhs?
    narginchk(2,3);

    % check if limb is a Limb
    if ~isa(limb, 'Limb'),
        error('TRG:plot:NonConsistentValue',...
                'Wrong input argument, expected a Limb object.');
    end
    if ~isa(q, 'numeric'),
        error('TRG:plot:NonConsistentValue', ...
                'Wrong input argument, expected a configuration vector (q).');
    end
    [tjPnts, nJtns] = size(q);
    if nJtns ~= limb.descr.n,
        error('TRG:plot:NonConsistentValue', ...
                'Wrong input argument, expected a configuration vector (q) with %d columns.',...
                    limb.descr.n);
    end

    % check for optional plot options
    PlotOpt = GrOptPlot;
    if nargin > 2,
        if ~isa(PlotOpt_, 'GrOptPlot'),
            error('TRG:plot:NonConsistentValue', ...
                    'Wrong input argument, expected a GrOptPlot object.');
        end
        PlotOpt = PlotOpt_;
    end

    % do plot!
    retval = limb.basePlot(q(1,:), PlotOpt);

    % animate!
    if tjPnts > 1,
        for i = 2:tjPnts,
            clf;
            retval = limb.basePlot(q(i,:), PlotOpt);
            pause(0.005);
        end
    end

    if(strcmp(PlotOpt.labels,'on'))
        F = limb.descr.P(:,1) + [0 0 -2]';
        fh=text(F(1),F(2),F(3),limb.name,'FontName','FixedWidth');
        set(fh,'fontWeight','bold');
    end
