% ContactStructure. Bind instances of Hand-Side POIs And Object-Side POIs with Contact Models
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Matteo Morelli, I.R.C. "E. Piaggio", University of Pisa
%
%   May 2012
%   Modified on July 2013
%       - isolateSingleContributionsFrom (TODO)
%       - error checking

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

classdef ContactStructure < handle

    properties (SetAccess = protected)
        matches;
        contactModels;
    end
    properties (Dependent)
        nContactModels;
    end
    methods (Static, Access = protected)
        function [alrMod, alrModIdx] = areThereAlreadyModelledMatches(mtc, mtcr)
            alrMod = false;
            alrModIdx = zeros(mtcr,1);
            for i=1:mtcr-1,
                if any(ismember(mtc(i,:), mtc(i+1:end,:), 'rows')),
                    alrModIdx(i) = i;
                    alrMod = true;
                end
            end
            alrModIdx(~alrModIdx) = [];
        end
    end
    methods
        function obj = ContactStructure(mtc, cMdl)
            if nargin < 2,
                narginchk(1,1);
                if ~isa(mtc, 'ContactStructure'),
                    error('TRG:ContactStructure:NonConsistentValue', ...
                        'Wrong input argument, expected a ContactStructure.');
                end
                if length(mtc) ~= 1,
                    error('TRG:ContactStructure:NonConsistentValue', ...
                        'Wrong input argument: expected a ContactStructure, not an array of ContactStructures.');
                end
                % copy
                obj.matches = mtc.matches;
                obj.contactModels = mtc.contactModels.copy;
                return;
            end
            narginchk(2,2);
            if ~isnumeric(mtc),
                error('TRG:ContactStructure:NonConsistentValue', ...
                	'Wrong input argument, expected a matrix of match indices.');
            end
            mtc = real(mtc);
            [mtcr, mtcc] = size(mtc);
            if mtcc ~= 2,
                error('TRG:ContactStructure:NonConsistentValue', ...
                    'Wrong input argument: expected 2 columns for the matrix of match indices.');
            end
            if ~isa(cMdl, 'Contact'),
                error('TRG:ContactStructure:NonConsistentValue', ...
                	'Wrong type of 2nd argument, expected a Contact model.');
            end
            cMdl = cMdl(:);
            cMdlr = numel(cMdl);
            if mtcr ~= cMdlr,
                error('TRG:ContactStructure:NonConsistentValue', ...
                    'The matrix of Matches and the vector of Contact models must have the same number of elements.');
            end
            % 
            % checking for mtc entries that are <= 0
            [mtcErrr, mtcErrc] = find(mtc <= 0);
            if ~isempty(mtcErrr),
                error('TRG:ContactStructure:NonConsistentValue', ...
                    ['Entries [', sprintf('%d %d, ', [mtcErrr, mtcErrc]') , '] are <= 0.']);
            end
            % checking for already modelled matches (ie, 2 or more identic rows in mtc)
            [alrMod, alrModIdx] = ContactStructure.areThereAlreadyModelledMatches(mtc, mtcr);
            if any(alrMod),
                error('TRG:ContactStructure:NonConsistentValue', ...
                    ['The following rows in Matches are duplicated: ', sprintf('%d,', alrModIdx)]);
            end
            obj.matches = mtc;
            obj.contactModels = cMdl.copy;
        end

        function obj2 = copy(obj)
            obj2 = ContactStructure(obj);
        end

        function display(obj)
            disp(' ');
            disp([inputname(1),' = '])
            disp(' ');
            for i = 1:length(obj),
                disp(char(char(obj(i)), ' '));
            end
        end

        function s = char(obj)
            s =         '+--------------+-------------+---------';
            s = char(s, '| POI# (HwGlv) | POI# (OwCvr)| Contact ');
            s = char(s, '+--------------+-------------+---------');
            for i=1:obj.nContactModels,
                s = char(s, ...
                sprintf('|      %d       |      %d      | %s  ', ...
                    obj.matches(i,:), obj.contactModels(i).typeFormatLong));
            end
            s = char(s, '+--------------+-------------+---------');
            w = find(s(1,:)=='-', 1, 'last');
            z = [char(45*ones(1,size(s,2) - w - 1)),'+'];
            s(1,w+1:end) = z;
            s(3,w+1:end) = z;
            s(end,w+1:end) = z;
            s([2,4:end-1],end) = '|';
        end

        function v = get.nContactModels(obj)
            v = numel(obj.contactModels);
        end

        function d = nTransmittedDofs(obj, reducedProb)
            narginchk(1,2);
            rp = GrOpt2dProblem;
            if nargin > 1,
                if ~isa(reducedProb, 'GrOpt2dProblem'),
                    error('TRG:Contact:NonConsistentValue', ...
                        'Wrong value specified for ''reducedProb'', expected a GrOpt2dProblem object.');
                end
                rp = reducedProb;
            end
            t = obj.contactModels.transmittedDofs(rp);
            d = sum([t{:}]);
        end

        function [fcll, mcll] = isolateSingleContributionsFrom(obj, mv, reducedProb, zeroSubst)
            % preliminary checks
            narginchk(2,4);
            rp = GrOpt2dProblem;
            zsub = false;
            if nargin > 2,
                if ~isa(reducedProb, 'GrOpt2dProblem'),
                    error('TRG:ContactStructure:NonConsistentValue', ...
                        'Wrong value specified for ''reducedProb'', expected a GrOpt2dProblem object.');
                end
                rp = reducedProb;
                if nargin > 3,
                    if ~isa(zeroSubst, 'logical'),
                        error('TRG:ContactStructure:NonConsistentValue', ...
                            'Wrong value specified for ''zeroPad'', expected a logical (true/false).');
                    end
                    zsub = zeroSubst;
                end
            end
            t = obj.contactModels.transmittedDofs(rp); % efficiency ( do not use nTransmittedDofs() )
            if size(mv,1) ~= sum([t{:}]),                       %
                error('TRG:ContactStructure:NonConsistentValue', ...
                        'The number of force/moment components is not consistent with the DoFs transmitted by the contact models, expected %d components instead of %d.', ...
                            sum([t{:}]), size(mv,1));
            end

            % how many contacts?
            n = obj.nContactModels;

            % return data structs
            fcll = cell(n,1); mcll = fcll;

            % for each contact model ...
            for i = 1:obj.nContactModels,
                % how many transmitted dofs?
                tfm = obj.contactModels(i).forceMomentTransmittedDofs(rp);

                % force components into fcll
                fcll{i} = mv(1:tfm(1),:);
                if zsub, if isempty(fcll{i}), fcll{i} = 0; end; end;

                % clear those components from mv
                mv(1:tfm(1),:) = [];

                % moment components into mcll
                mcll{i} = mv(1:tfm(2),:);
                if zsub, if isempty(mcll{i}), mcll{i} = 0; end; end;

                % clear those components from mv
                mv(1:tfm(2),:) = [];
            end
        end
    end

end