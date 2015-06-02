% CONTACTSTRUCTURE.PLOT. Plot the Relevant Info of a Contact Model
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Nicola Greco, I.R.C. "E. Piaggio", University of Pisa
%   Matteo Morelli, I.R.C. "E. Piaggio", University of Pisa
%
%   August 2013

% Copyright (C) 2013 Interdepartmental Research Center "E. Piaggio", University of Pisa
%
% This file is part of THE Robotic Grasping Toolbox for use with MATLAB(R).
%
% THE Robotic Grasping Toolbox for use with MATLAB(R) is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
%
% THE Robotic Grasping Toolbox for use with MATLAB(R) is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with THE Robotic Grasping Toolbox for use with MATLAB(R).  If not, see <http://www.gnu.org/licenses/>.

function plot(cst, T, PlotOpt_, reducedProb)

    % rhs args ?
    narginchk(2,4);

    % basic checks
    if ~isa(cst, 'ContactStructure'),
        error('TRG:plot:NonConsistentValue', ...
            'Wrong input argument, expected a ContactStructure object.');
    end
    if ~iscell(T),
        error('TRG:plot:NonConsistentValue', ...
            'Wrong input argument, expected a cell array of POI frames.');
    end
    [Trows, Tcols] = cellfun(@size, T);
    if ~all([Trows,Tcols] == 4),
        error('TRG:plot:NonConsistentValue', ...
            'Wrong input argument, the cell array must contain POI frames.');
    end
    PlotOpt = GrOptPlot;  % builds standard plot options
    if nargin > 2,
        if ~isa(PlotOpt_, 'GrOptPlot'),
            error('TRG:plot:NonConsistentValue', ...
                'Wrong input argument, expected a GrOptPlot object.');
        end
        PlotOpt = PlotOpt_;
        if nargin > 3,
            if ~isa(reducedProb, 'GrOpt2dProblem'),
                error('TRG:plot:NonConsistentValue', ...
                    'Wrong input argument, expected a GrOpt2dProblem object.');
            end
            rp = reducedProb;
        end
    end

    % plot
    %
    % axes
    if isempty(findobj(gcf,'type','axes'))
        ax = axes;
        grid on;
        axis equal;

        if isempty(PlotOpt.camView)
            view(3)
        else
            view(PlotOpt.camView);
        end
        
        if isempty(PlotOpt.Xlimit)
            set(ax,'XLimMode','auto');
        else
            set(ax,'xlim',PlotOpt.Xlimit);
        end
        if isempty(PlotOpt.Ylimit)
            set(ax,'YLimMode','auto');
        else
            set(ax,'ylim',PlotOpt.Ylimit);
        end
        if isempty(PlotOpt.Zlimit)
            set(ax,'ZLimMode','auto');
        else
            set(ax,'zlim',PlotOpt.Zlimit);
        end
        
        xlabel(PlotOpt.Xlabel);ylabel(PlotOpt.Ylabel);zlabel(PlotOpt.Zlabel);
    else
        ax = gca;
    end
    %
    % contact info
    for i = 1 : cst.nContactModels,
        k = cst.matches(i,1);   % contact index, hand side
        if(strcmp(PlotOpt.contactPlanes,'on')),
            srv_plot_contact_plane(T{k});
        end
        if(strcmp(PlotOpt.frictionCones,'on')),
            cl = cst.contactModels(i);
            Cni = rp.frameNormal(cl.contactFrameObjectSide);
            if rp.isSet,
                Cni = rp.restoreVectorTo3d(Cni);
            end
            fc_handle = srv_plot_friction_cone(cl.frictionCoefficients(1), Cni, PlotOpt.frictionConeScale); % linear friction (mu)
            set(fc_handle,'Matrix',T{k});
        end
    end

end
