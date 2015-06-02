% GrOptPlot. GrOptPlot Class
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Nicola Greco, I.R.C. "E. Piaggio", University of Pisa
%
%   March 2013

% Copyright (C) 2012, 2013 Interdepartmental Research Center "E. Piaggio", University of Pisa
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

classdef GrOptPlot < handle

    properties
        linksColor;
        jointsColor;
        objectColor;
        linksRadius;
        jointsRadius;
        scaling;
        frameScale;
        frameOriginMarkerSize;
        Xlabel;
        Ylabel;
        Zlabel;
        Xlimit;
        Ylimit;
        Zlimit;
        camView;
        contactPlanes;
        frictionCones;
        frictionConeScale;
        jointsAxis;
        labels;
        
    end
   
    methods
        function obj = GrOptPlot(varargin)
            obj.linksColor = [0.3 0.3 1];
            obj.jointsColor = [1 0 0];
            obj.objectColor = 'r';
            obj.linksRadius = 1;
            obj.jointsRadius = 1;
            obj.scaling = 1;
            obj.frameScale = 15;
            obj.frameOriginMarkerSize = 50;
            obj.Xlabel = 'X';
            obj.Ylabel = 'Y';
            obj.Zlabel = 'Z';
            obj.Xlimit = [];
            obj.Ylimit = [];
            obj.Zlimit = [];
            obj.camView = [0,0];
            obj.contactPlanes = 'off';
            obj.frictionCones = 'off';
            obj.frictionConeScale = 1;
            obj.jointsAxis = 'off';
            obj.labels = 'off';
            
            if nargin == 1,
                if isa(varargin{1}, 'GrOptPlot'),
                    if length(varargin{1}) ~= 1,
                        error('TRG:GrOptPlot:NonConsistentValue', ...
                                'Wrong input argument: expected a single GrOptPlot object, not an array of objects.');
                    end
                    a1 = varargin{1};
                    obj.linksColor = a1.linksColor;
                    obj.jointsColor = a1.jointsColor;
                    obj.objectColor = a1.objectColor;
                    obj.linksRadius = a1.linksRadius;
                    obj.jointsRadius = a1.jointsRadius;
                    obj.scaling = a1.scaling;
                    obj.frameScale = a1.frameScale;
                    obj.frameOriginMarkerSize = a1.frameOriginMarkerSize;
                    obj.Xlabel = a1.Xlabel;
                    obj.Ylabel = a1.Ylabel;
                    obj.Zlabel = a1.Zlabel;
                    obj.Xlimit = a1.Xlimit;
                    obj.Ylimit = a1.Ylimit;
                    obj.Zlimit = a1.Zlimit;
                    obj.camView = a1.camView;
                    obj.contactPlanes = a1.contactPlanes;
                    obj.frictionCones = a1.frictionCones;
                    obj.frictionConeScale = a1.frictionConeScale;
                    obj.jointsAxis = a1.jointsAxis;
                    obj.labels = a1.labels;
                end
            end
            if nargin > 1,
                % varargin
                i = 1;
                while i < length(varargin),
                    processedProp = varargin{i};
                    switch processedProp,
                        case {'linksColor','jointsColor','objectColor'},
                            i = i + 1;
                            if isa(varargin{i}, 'numeric'), % RGB notation
                                if(size(varargin{i},1)~= 1 || size(varargin{i},2)~= 3 || (norm(varargin{i}) > 1.7321))
                                    error('TRG:GrOptPlot:NonConsistentValue', ...
                                    'wrong value specified for ''%s'', expected a 1x3 vector with elements between 0.0 and 1.0',...
                                    processedProp);

                                end
                            else
                                if isa(varargin{i}, 'char')
                                    if(size(varargin{i},1)~= 1)
                                        error('TRG:GrOptPlot:NonConsistentValue', ...
                                        'wrong value specified for ''%s'', expected a single string ',...
                                        processedProp);
                                    end
                                else
                                error('TRG:GrOptPlot:NonConsistentValue', ...
                                        'unknown class specified for ''%s'' ',...
                                        processedProp);
                                end
                            end
                            
                            eval(['obj.', processedProp, ' = varargin{i};']);
                        
                            
                        case {'linksRadius','jointsRadius','scaling','frameScale','frameOriginMarkerSize','frictionConeScale'}
                            i = i + 1;
                            if isa(varargin{i}, 'numeric')
                                if(size(varargin{i},1)~= 1 || size(varargin{i},2)~= 1)
                                    error('TRG:GrOptPlot:NonConsistentValue', ...
                                        'wrong value specified for ''%s'', expected single numeric value, not an array',...
                                        processedProp);
                                end
                            else
                                error('TRG:GrOptPlot:NonConsistentValue', ...
                                        'wrong value specified for ''%s'', expected a numeric value ',...
                                        processedProp);
                            end
                            
                            eval(['obj.', processedProp, ' = abs(varargin{i});']);
                        
                        
                        case {'Xlabel','Ylabel','Zlabel'},
                            i = i + 1;
                            if isa(varargin{i}, 'char')
                                if(size(varargin{i},1)~= 1)
                                    error('TRG:GrOptPlot:NonConsistentValue', ...
                                    'wrong value specified for ''%s'', expected a single string ',...
                                    processedProp);
                                end
                            else
                                error('TRG:GrOptPlot:NonConsistentValue', ...
                                    'wrong value specified for ''%s'', expected a string ',...
                                    processedProp);
                            end
                            
                            eval(['obj.', processedProp, ' = varargin{i};']);
                            
                        case {'Xlimit','Ylimit','Zlimit'},
                            i = i + 1;
                            if isa(varargin{i}, 'numeric')
                                if(size(varargin{i},1)~= 2)
                                    error('TRG:GrOptPlot:NonConsistentValue', ...
                                    'wrong value specified for ''%s'', expected a 1x2 numeric array ',...
                                    processedProp);
                                end
                            else
                                error('TRG:GrOptPlot:NonConsistentValue', ...
                                    'wrong value specified for ''%s'', expected a 1x2 numeric array ',...
                                    processedProp);
                            end
                            
                            eval(['obj.', processedProp, ' = varargin{i};']);
                            
                            
                        case 'camView'
                            i = i + 1;
                            if isa(varargin{i}, 'numeric')
                                if(size(varargin{i},1)~= 1 || size(varargin{i},2)~= 1)
                                    error('TRG:GrOptPlot:NonConsistentValue', ...
                                    'wrong value specified for ''%s'', expected a 1x2 numeric array ',...
                                    processedProp);
                                end
                            else
                                error('TRG:GrOptPlot:NonConsistentValue', ...
                                    'wrong value specified for ''%s'', expected 1x2 numeric array ',...
                                    processedProp);
                            end
                            
                            obj.camView = varargin{i};
                            
                        case {'contactPlanes', 'frictionCones', 'jointsAxis','labels'}
                            i = i + 1;
                            switch varargin{i}
                                case {'on','off'}
                                    eval(['obj.', processedProp, ' = varargin{i};']);
                                otherwise
                                    error('TRG:GrOptPlot:NonConsistentValue', ...
                                    'wrong value specified for ''%s'', expected ''on'' or ''off'' ',...
                                    processedProp);
                            end
                            
                        otherwise,
                            error('TRG:GrOptPlot:UnknownProperty', ...
                                'unknown property passed to GrOptPlot ''%s''', processedProp);

                     end
                    i = i + 1;
                end
            end
        end  % constructor end

        
        function obj2 = copy(obj)
            obj2 = GrOptPlot(obj);
        end
        
        
        function set.linksColor(obj,val)
            if isa(val, 'numeric'), % RGB notation
                if(size(val,1)~= 1 || size(val,2)~= 3 || (norm(val) > 1.7321))
                    error('TRG:GrOptPlot:NonConsistentValue', ...
                    'wrong value specified for linksColor, expected a 1x3 vector with elements between 0.0 and 1.0');
                end
            else
                if isa(val, 'char') % string notation
                    if(size(val,1)~= 1)
                        error('TRG:GrOptPlot:NonConsistentValue', ...
                        'wrong value specified for linksColor, expected a single string ');
                    end
                else
                    error('TRG:GrOptPlot:NonConsistentValue', ...
                    'unknown class specified for linksColor');
                end
            end
            obj.linksColor = val;
        end
        
        function set.jointsColor(obj,val)
            if isa(val, 'numeric'), % RGB notation
                if(size(val,1)~= 1 || size(val,2)~= 3 || (norm(val) > 1.7321))
                    error('TRG:GrOptPlot:NonConsistentValue', ...
                    'wrong value specified for jointsColor, expected a 1x3 vector with elements between 0.0 and 1.0');
                end
            else
                if isa(val, 'char') % string notation
                    if(size(val,1)~= 1)
                        error('TRG:GrOptPlot:NonConsistentValue', ...
                        'wrong value specified for jointsColor, expected a single string ');
                    end
                else
                    error('TRG:GrOptPlot:NonConsistentValue', ...
                    'unknown class specified for jointsColor');
                end
            end
            obj.jointsColor = val;
        end
        
        function set.objectColor(obj,val)
            if isa(val, 'numeric'), % RGB notation
                if(size(val,1)~= 1 || size(val,2)~= 3 || (norm(val) > 1.7321))
                    error('TRG:GrOptPlot:NonConsistentValue', ...
                    'wrong value specified for objectColor, expected a 1x3 vector with elements between 0.0 and 1.0');
                end
            else
                if isa(val, 'char') % string notation
                    if(size(val,1)~= 1)
                        error('TRG:GrOptPlot:NonConsistentValue', ...
                        'wrong value specified for objectColor, expected a single string ');
                    end
                else
                    error('TRG:GrOptPlot:NonConsistentValue', ...
                    'unknown class specified for objectColor');
                end
            end
            obj.objectColor = val;
        end
        
        function set.linksRadius(obj,val)
            if isa(val, 'numeric')
                if(size(val,1)~= 1 || size(val,2)~= 1)
                    error('TRG:GrOptPlot:NonConsistentValue', ...
                    'wrong value specified for linksRadius, expected a single numeric value, not an array');
                end
            else
                error('TRG:GrOptPlot:NonConsistentValue', ...
                'wrong value specified for linksRadius, expected a numeric value ');
            end
            obj.linksRadius = val;
        end
        
        function set.jointsRadius(obj,val)
            if isa(val, 'numeric')
                if(size(val,1)~= 1 || size(val,2)~= 1)
                    error('TRG:GrOptPlot:NonConsistentValue', ...
                    'wrong value specified for jointsRadius, expected a single numeric value, not an array');
                end
            else
                error('TRG:GrOptPlot:NonConsistentValue', ...
                'wrong value specified for jointsRadius, expected a numeric value ');
            end
            obj.jointsRadius = abs(val);
        end
        
        function set.scaling(obj,val)
            if isa(val, 'numeric')
                if(size(val,1)~= 1 || size(val,2)~= 1)
                    error('TRG:GrOptPlot:NonConsistentValue', ...
                    'wrong value specified for scaling, expected a single numeric value, not an array');
                end
            else
                error('TRG:GrOptPlot:NonConsistentValue', ...
                'wrong value specified for scaling, expected a numeric value ');
            end
            obj.scaling = abs(val);
        end

        function set.frictionConeScale(obj,val)
            if isa(val, 'numeric')
                if(size(val,1)~= 1 || size(val,2)~= 1)
                    error('TRG:GrOptPlot:NonConsistentValue', ...
                    'wrong value specified for scaling of friction cone, expected a single numeric value, not an array');
                end
            else
                error('TRG:GrOptPlot:NonConsistentValue', ...
                'wrong value specified for scaling of friction cone, expected a numeric value ');
            end
            obj.frictionConeScale = abs(val);
        end

        function set.frameScale(obj,val)
            if isa(val, 'numeric')
                if(size(val,1)~= 1 || size(val,2)~= 1)
                    error('TRG:GrOptPlot:NonConsistentValue', ...
                    'wrong value specified for frame scale, expected a single numeric value, not an array');
                end
            else
                error('TRG:GrOptPlot:NonConsistentValue', ...
                'wrong value specified for frame scale, expected a numeric value ');
            end
            obj.frameScale = abs(val);
        end

        function set.frameOriginMarkerSize(obj,val)
            if isa(val, 'numeric')
                if(size(val,1)~= 1 || size(val,2)~= 1)
                    error('TRG:GrOptPlot:NonConsistentValue', ...
                    'wrong value specified for the marker size of frame origin scaling, expected a single numeric value, not an array');
                end
            else
                error('TRG:GrOptPlot:NonConsistentValue', ...
                'wrong value specified for marker size of frame origin, expected a numeric value ');
            end
            obj.frameOriginMarkerSize = abs(val);
        end

        function set.Xlabel(obj,val)
            if isa(val, 'char')
                if(size(val,1)~= 1)
                    error('TRG:GrOptPlot:NonConsistentValue', ...
                    'wrong value specified for Xlabel, expected a single string ');
                end
            else
                error('TRG:GrOptPlot:NonConsistentValue', ...
                'wrong value specified for Xlabel, expected a string ');
            end
            
            obj.Xlabel = val;
        end
         
        function set.Ylabel(obj,val)
            if isa(val, 'char')
                if(size(val,1)~= 1)
                    error('TRG:GrOptPlot:NonConsistentValue', ...
                    'wrong value specified for Ylabel, expected a single string ');
                end
            else
                error('TRG:GrOptPlot:NonConsistentValue', ...
                'wrong value specified for Ylabel, expected a string ');
            end
            
            obj.Ylabel = val;
        end
         
        function set.Zlabel(obj,val)
            if isa(val, 'char')
                if(size(val,1)~= 1)
                    error('TRG:GrOptPlot:NonConsistentValue', ...
                    'wrong value specified for Zlabel, expected a single string ');
                end
            else
                error('TRG:GrOptPlot:NonConsistentValue', ...
                'wrong value specified for Zlabel, expected a string ');
            end
            
            obj.Zlabel = val;
        end
        
        function set.contactPlanes(obj,val)
            switch val
                case {'on','off'}
                    obj.contactPlanes = val;
                
                otherwise
                    error('TRG:GrOptPlot:NonConsistentValue', ...
                    'wrong value specified for contactPlanes, expected ''on'' or ''off'' ');
            end
        end
        
        function set.frictionCones(obj,val)
            switch val
                case {'on','off'}
                    obj.frictionCones = val;
                
                otherwise
                    error('TRG:GrOptPlot:NonConsistentValue', ...
                    'wrong value specified for frictionCones, expected ''on'' or ''off'' ');
            end
        end
        
        function set.jointsAxis(obj,val)
            switch val
                case {'on','off'}
                    obj.jointsAxis = val;
                
                otherwise
                    error('TRG:GrOptPlot:NonConsistentValue', ...
                    'wrong value specified for jointsAxis, expected ''on'' or ''off'' ');
            end
        end
       
        function set.labels(obj,val)
            switch val
                case {'on','off'}
                    obj.labels = val;
                
                otherwise
                    error('TRG:GrOptPlot:NonConsistentValue', ...
                    'wrong value specified for labels, expected ''on'' or ''off'' ');
            end
        end

        function obj = convertFromRtbFormat(obj, rtbOpt)
            obj.Xlimit = rtbOpt.workspace([1,4]);
            obj.Ylimit = rtbOpt.workspace([2,5]);
            obj.Zlimit = rtbOpt.workspace([3,6]);

            onOffVals = {'off', 'on'};
            obj.jointsAxis = onOffVals{rtbOpt.jaxes + 1};
        end

    end
end
