%Hand.PLOT. Perform 3D Graphical Animation of an Hand Model for a Given Joint-Trajectory
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Matteo Morelli, I.R.C. "E. Piaggio", University of Pisa
%
%   August 2012
%   Modified on August 2013

% Copyright (C) 2012, 2013 Interdepartmental Research Center "E. Piaggio", University of Pisa
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

function conf = plot(hnd, q, gpPlot_)

    % rhs args?
    narginchk(2,3);

    % init return val
    conf = [];

    % basic checks
    if ~isa(hnd, 'Hand'),
        error('TRG:plot:NonConsistentValue', ...
            'Wrong input argument, expected a Hand object.');
    end
    if ~isa(q, 'numeric'),
        error('TRG:plot:NonConsistentValue', ...
            'Wrong input argument, expected a configuration vector (q).');
    end
    PlotOpt = GrOptPlot;  % builds standard plot options
    if nargin > 2,
        if ~isa(gpPlot_, 'GrOptPlot'),
            if ~isa(gpPlot_, 'struct'),
                error('TRG:plot:NonConsistentValue', ...
                    'Wrong input argument, expected a GrOptPlot object.');
            end
            % process (split) options
            [optHand, optLimb] = process_opt_argum(hnd, hnd.nLimbs, gpPlot_);
            PlotOpt.convertFromRtbFormat(optLimb);
        else
            PlotOpt = gpPlot_;
        end
    end

    % DoFs, finger numbers etc.
    jntNo = hnd.dofs;
    [pntNo, qCols] = size(q);
    if qCols ~= jntNo,
        error('TRG:plot:NonConsistentValue', ...
            'Insufficient columns in q, expected %d.', jntNo);
    end

    % do plot
    conf = hnd.basePlot(q(1,:), PlotOpt);

    % animate!
    if pntNo > 1,
        for i = 2:pntNo,
            clf;
            conf = hnd.basePlot(q(i,:), PlotOpt);
            pause(0.005);
        end
    end

    if(strcmp(PlotOpt.labels,'on'))
        F = [0 0 -2]';
        fh=text(F(1),F(2),F(3),hnd.name,'FontName','FixedWidth');
        set(fh,'fontWeight','bold');
    end

end

function [optHand, optLimb] = process_opt_argum(hnd, fgrNo, hoPlot)
    %
    % process hoPlot (TODO)
    % [hndCellOpAr, lmbCellOpAr] = hoPlot.process();
    %
    hndCellOpAr = {};
    lmbCellOpAr = {'nobase'};

    % Limb options
    % ============
    optLimb = [];
	optdim = []; optdiM = []; optmag = [];
    for i = 1:fgrNo,
        ithLimb = hnd.limbs(i);
        optLimb = ithLimb.plot(lmbCellOpAr); % get SerialLink plot options
        optdim = [optdim; optLimb.workspace(1:2:end)];
        optdiM = [optdiM; optLimb.workspace(2:2:end)];
        optmag = [optmag; optLimb.magscale];
    end
    optdim = min(optdim, [], 1);
    optdiM = max(optdiM, [], 1);
    optmag = max(optmag);

	% Set uniform values for:
    % axes data bounds and mags
    works = [optdim; optdiM];
    eqIdx = find( works(1,:) == works(2,:) );
    if ~isempty(eqIdx),
        works(2,eqIdx) = works(2,eqIdx) + 0.1;
    end
    optLimb.workspace = works(:)';
    optLimb.magscale = optmag;

    % projections (perspective/orthographic)
    ... % TODO

    % Hand options
    % ============
    optHand = [];
    ... % TODO
end
