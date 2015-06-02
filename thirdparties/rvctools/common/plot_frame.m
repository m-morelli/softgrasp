%TRPLOT Plot a coordinate frame represented by a homogeneous transformation
%
%  trplot(T, options) draws a coordinate frame corresponding to the homogeneous 
%  transformation T.
%
% Options::
%  'color', c    Specify color of the axes, Matlab colorspec
%  'axes'
%  'axis'
%  'name', n     Specify the name of the coordinate frame, 
%  'framename', n
%  'text_opts', to
%  'view', v     Specify the view angle for the Matlab axes
%  'width', w
%  'arrow'      
%  'length', l  Specify length of the axes (default 1)
%
% Examples:
%    trplot( T, 'color', 'r');
%    trplot( T, 'color', 'r', 'name', 'B')

% Name can be a string which is appears in the axis labels, X<name>, etc
%   eg. trplot(trotx(.2), 'cam')
%
% Name can be a cell array {axname, framename}, axname acts the same way
% as above, framename is written near the origin.  We can use LaTeX math
% mode.  For axname include a %c where the axis name (X, Y, Z) will be
% substituted  
%  eg.  trplot(trotx(.2), {'$%c_{cam}$','$\{X_{cam}\}$'})
%


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

% TODO
%  need to decide how to handle scaling
%  what does hold on mean?  don't touch scaling?

function hout = plot_frame(T, varargin)

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
    opt.length = 1;
    argc = 1;
    while argc <= length(varargin)
        switch lower(varargin{argc})
        case 'color'
            opt.color = varargin{argc+1}; argc = argc+1;
        case 'frame'
            opt.framename = varargin{argc+1}; argc = argc+1;
        case 'label'
            opt.axlabel = varargin{argc+1}; argc = argc+1;
        case 'length'
            opt.length = varargin{argc+1}; argc = argc+1;
        case 'noaxes'
            opt.axes = false;
        case 'arrow'
            opt.arrow = true;
        case 'unit'
            opt.axis = [-1 1 -1 1 -1 1]*1.2';
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

    if isempty(opt.axis)
        if all(size(T) == [3 3]) || norm(transl(T)) < eps
            opt.axis = [-1 1 -1 1 -1 1]*1.2';
        end
            
    end
    
    % TODO: should do the 2D case as well
    
    ih = ishold;
    hold on

    opt.text_opts = {opt.text_opts{:}, 'Color', opt.color};

	hold on

    % trplot( Q.r, fmt, color);
    if size(T) == [3 3],
        T = r2t(T);
    end


	% create unit vectors
	o =  homtrans(T, [0 0 0]');
	x1 = homtrans(T, [opt.length 0 0]');
	y1 = homtrans(T, [0 opt.length 0]');
	z1 = homtrans(T, [0 0 opt.length]');

    
    % draw the axes
    
    mstart = [o o o]';
    mend = [x1 y1 z1]';
    
    hg = hgtransform;

    if opt.arrow
        % draw the 3 arrows
        S = [opt.color num2str(opt.width)];
        ha = arrow3(mstart(:,1:3), mend(:,1:3), S);
        for h=ha'
            set(h, 'Parent', hg);
        end
    else
        for i=1:3,
            h = plot2([mstart(i,1:3); mend(i,1:3)], opt.color);
            set(h, 'Parent', hg);

        end
    end

    % label the axes
    if isempty(opt.framename),
        fmt = '%c';
    else
%             if opt.axlabel(1) == '$',
%                 fmt = axlabel;
%                 opt.text_opts = {opt.text_opts{:}, 'Interpreter', 'latex'};
%             else,
%                 fmt = sprintf('%%c%s', axlabel);
%             end
        fmt = sprintf('%%c_{%s}', opt.framename);
    end
    
    % add the labels to each axis
	h = text(x1(1), x1(2), x1(3), sprintf(fmt, 'X'));
	set(h, opt.text_opts{:});
    set(h, 'Parent', hg);
   

	h = text(y1(1), y1(2), y1(3), sprintf(fmt, 'Y'));
	set(h, opt.text_opts{:});
        set(h, 'Parent', hg);
    

	h = text(z1(1), z1(2), z1(3), sprintf(fmt, 'Z'));
	set(h, opt.text_opts{:});
        set(h, 'Parent', hg);
    
    % label the frame
    if ~isempty(opt.framename),
        h = text(o(1)-0.04*x1(1), o(2)-0.04*y1(2), o(3)-0.04*z1(3), ...
            ['\{' opt.framename '\}']);
        set(h, 'VerticalAlignment', 'middle', ...
            'HorizontalAlignment', 'center', opt.text_opts{:});
    end
    
    if ~opt.axes
        set(gca, 'visible', 'off');
    end
    if isstr(opt.view) && strcmp(opt.view, 'auto')
        cam = x1+y1+z1;
        view(cam(1:3));
    elseif ~isempty(opt.view)
        view(opt.view);
    end
	grid on
	if ~ih
		hold off
    end
    
    if nargout > 0
        hout = hg;
    end
