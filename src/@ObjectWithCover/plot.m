% ObjectWithCover.PLOT. Plot a ObjectWithCover for a Given Configuration
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

function plot(cvr, u, PlotOpt_)

    % rhs args ?
    narginchk(2,3);

    % basic checks
    if ~isa(cvr, 'ObjectWithCover'),
        error('TRG:plot:NonConsistentValue', ...
            'Wrong input argument, expected a ObjectWithCover object.');
    end
    if ~isa(u, 'numeric'),
        error('TRG:plot:NonConsistentValue', ...
            'Wrong input argument, expected an object configuration vector (u).');
    end
    u = real(u(:));
    if numel(u) ~= 6,
        error('TRG:plot:NonConsistentValue', ...
                'Wrong input argument, expected a configuration vector (u) with 6 elements.');
    end
    PlotOpt = GrOptPlot;  % builds standard plot options
    if nargin > 2,
        if ~isa(PlotOpt_, 'GrOptPlot'),
            error('TRG:plot:NonConsistentValue', ...
                'Wrong input argument, expected a GrOptPlot object.');
        end
        PlotOpt = PlotOpt_;
    end

    % structs
    com = cvr.object;
    cps = cvr.POIStructure;

    % frames
    %
    % object (baricentric frame)
    objFkineTr = com.fkine(u);
    %
    % POIs
    T = cvr.poiForwardKinematics(u);

    % do plot!
    %
    % axes
    if isempty(findobj(gcf,'type','axes'))
        ax = axes;
        title(cvr.name);
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
        camlight left;
    else
        ax = gca;
    end
    %
    % object (frame)
    grid on;
    srv_drawframe(objFkineTr, com.name, PlotOpt);
    %
    % POIs (frames)
    for i = 1 : cps.nPois,
        srv_drawframe(T{i}, cps.pois(i).name, PlotOpt);
    end

end
