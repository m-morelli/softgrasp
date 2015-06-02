%TRPLOT2 Plot a planar transformation
%
% TRPLOT2(T, options) draws a 2D coordinate frame represented by the homogeneous 
% transform T.
%
% Options::
% 'color', c         The color to draw the axes, Matlab colorspec.
% 'axes'             Set dimensions of the Matlab axes
% 'frame', f         The name which appears on the axis labels and the frame itself
% 'text_opts', opt   A cell array of Matlab text properties
% 'arrow'            Use arrows rather than line segments for the axes
% 'width', w         Width of arrow tips
% 'handle', h        Draw in the Matlab axes specified by h
%
% See also TRPLOT.


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

%   'frame', name   name of the frame, used for axis subscripts and origin
%   'color', color  Matlab color specificication for the frame and annotations
%   'noaxes'        show the frame but no Matlab axes
%   'arrow'         use the contributed arrow3 function to draw the frame axes
%   'width', width  width of lines to draw if using arrow3

function hout = trplot(T, varargin)

    opt.color = 'b';
    opt.axes = true;
    opt.name = [];
    opt.axis = [];
    opt.framename = [];
    opt.axlabel = [];
    opt.text_opts = {};
    opt.view = [];
    opt.width = 1;
    opt.arrow = false;
    argc = 1;
    while argc <= length(varargin)
        switch lower(varargin{argc})
        case 'color'
            opt.color = varargin{argc+1}; argc = argc+1;
        case 'frame'
            opt.framename = varargin{argc+1}; argc = argc+1;
        case 'label'
            opt.axlabel = varargin{argc+1}; argc = argc+1;
        case 'noaxes'
            opt.axes = false;
        case 'arrow'
            opt.arrow = true;
        case 'unit'
            opt.axis = [-1 1 -1 1 ]*1.2';
        case 'axis'
            opt.axis = varargin{argc+1}; argc = argc+1;
        case 'textopts'
            opt.text_opts = varargin{argc+1}; argc = argc+1;
        case 'width'
            opt.width = varargin{argc+1}; argc = argc+1;
        case 'view'
            opt.view = varargin{argc+1}; argc = argc+1;
        otherwise
            error( sprintf('unknown option <%s>', varargin{argc}));
        end
        argc = argc + 1;
    end

    ih = ishold;
    if ~ih
        % if hold is not on, then clear the axes and set scaling
		cla
        if ~isempty(opt.axis)
            axis(opt.axis);
        end
        axis equal
        
        if opt.axes
            xlabel( 'X');
            ylabel( 'Y');
        end
        new_plot = true;
    else
        %set(gca, 'XLimMode', 'auto');
        %set(gca, 'YLimMode', 'auto');
	end

    opt.text_opts = {opt.text_opts{:}, 'Color', opt.color};

	hold on

	% create unit vectors
	o =  T*[0 0 1]'; o = o(1:2);
	x1 = T*[1 0 1]'; x1 = x1(1:2);
	y1 = T*[0 1 1]'; y1 = y1(1:2);
    
    % draw the axes
    
    mstart = [o o]';
    mend = [x1 y1]';
    
    if opt.arrow
        hg = hgtransform;
        % draw the 2 arrows
        S = [opt.color num2str(opt.width)];
        ha = arrow3(mstart(:,1:2), mend(:,1:2), S);
        for h=ha'
            set(h, 'Parent', hg);
        end
    else
        for i=1:2
            plot2([mstart(i,1:2); mend(i,1:2)], 'Color', opt.color);
        end
    end

    % label the axes
    if isempty(opt.framename)
        fmt = '%c';
    else
%             if opt.axlabel(1) == '$'
%                 fmt = axlabel;
%                 opt.text_opts = {opt.text_opts{:}, 'Interpreter', 'latex'};
%             else
%                 fmt = sprintf('%%c%s', axlabel);
%             end
        fmt = sprintf('%%c_{%s}', opt.framename);
    end
    
    % add the labels to each axis
	h = text(x1(1), x1(2), sprintf(fmt, 'X'));
	set(h, opt.text_opts{:});
    if opt.arrow
        set(h, 'Parent', hg);
    end

	h = text(y1(1), y1(2), sprintf(fmt, 'Y'));
	set(h, opt.text_opts{:});
    if opt.arrow
        set(h, 'Parent', hg);
    end

    % label the frame
    if ~isempty(opt.framename)
        h = text(o(1)-0.04*x1(1), o(2)-0.04*y1(2), ...
            ['\{' opt.framename '\}']);
        set(h, 'VerticalAlignment', 'middle', ...
            'HorizontalAlignment', 'center', opt.text_opts{:});
    end
    
    if ~opt.axes
        set(gca, 'visible', 'off');
    end
	grid on
	if ~ih
		hold off
    end
    
    if nargout > 0
        hout = hg;
    end
