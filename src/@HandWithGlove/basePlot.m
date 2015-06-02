% HandWithGlove.BASEPLOT. Plot a HandWithGlove for a Given Configuration
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
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

function basePlot(glv, q, PlotOpt_)

    % rhs args ?
    narginchk(2,3);

    % basic checks
    if ~isa(glv, 'HandWithGlove'),
        error('TRG:plot:NonConsistentValue', ...
            'Wrong input argument, expected a HandWithGlove object.');
    end
    if ~isa(q, 'numeric'),
        error('TRG:plot:NonConsistentValue', ...
            'Wrong input argument, expected a configuration vector (q).');
    end
    PlotOpt = GrOptPlot;  % builds standard plot options
    if nargin > 2,
        if ~isa(PlotOpt_, 'GrOptPlot'),
            error('TRG:plot:NonConsistentValue', ...
                'Wrong input argument, expected a GrOptPlot object.');
        end
        PlotOpt = PlotOpt_;
    end

    % hand plot
    glv.hand.plot(q,PlotOpt);

    % POIs plot
    gps = glv.POIStructure;
    T = glv.poiForwardKinematics(q);
    for i = 1 : gps.nPois,
        srv_drawframe(T{i}, gps.pois(i).name, PlotOpt);
    end

end
