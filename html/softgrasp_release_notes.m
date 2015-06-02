%% Upslope Area Toolbox Release Notes
% This example toolbox implements the Tarboton method for computing upslope
% area and related measurements on a digital elevation model (DEM).
%
% The Upslope Area Toolbox requires Image Processing Toolbox(TM).
% 
%% References
% Algorithm: David G. Tarboton, "A new method for the
% determination of flow directions and upslope areas in grid
% digital elevation models," _Water Resources Research_, vol. 33,
% no. 2, pp. 309-319, February 1997.
% http://www.engineering.usu.edu/cee/faculty/dtarb/96wr03137.pdf
%
% Reference: "Steve on Image Processing," upslope area algorithm development
% series. http://blogs.mathworks.com/steve/category/upslope-area/
%
% The latest version of this fully functional toolbox is available on
% <http://www.mathworks.com/matlabcentral/fileexchange/15818 MATLAB Central File
% Exchange>. 
%
%% Version 2.0, 09-Dec-2009
%
% * Modify |fillSinks| to handle input DEMs containing border NaNs.
% * Modify |visMap| to accept two input arguments, in which case no starting or
% ending DEM locations are shown in blue.
% * Functions renamed to use camel-case instead of underscores.  (To use the
% original function names, put the folder version_1 on the MATLAB path.)
% * New documentation written and integrated into MATLAB Help Browser.
%
%% Version 1.4.2, 16-Sep-2009
% 
% * Updated unit tests to use |assertElementsAlmostEqual|.
% 
%% Version 1.4.1, 10-Mar-2009
% 
% * Fixed help typo in |dependence_map|. 
% * Allowed input matrix E to contain NaNs in |postprocess_plateaus|. 
% * Added unit test directory.
% 
%% Version 1.4, 25-Sep-2008
% 
% * Modified |fill_sinks| to call |imfill| with 8-connectivity
%   instead of 4-connectivity.
% 
%% Version 1.3, 14-Feb-2008
% 
% * Improved handling of groups of NaNs that touch the DEM border
%   so that the |dependence_map| and |influence_map| calculations are
%   correct.  Now |flow_matrix| is computed so that border NaN pixels
%   have zero flow weights to and from all their neighbors.  As a
%   nice side effect of the change, |flow_matrix| is now faster
%   for datasets that have border NaN pixels.
% 
%% Version 1.2, 02-Oct-2007
% 
% * Changed handling of groups of NaNs that touch the DEM border. 
% * Added |border_nans| function.
% 
%% Version 1.1, 06-Aug-2007
% 
% * Incompatible change made to |upslope_area|.  This function no longer
%   "flattens" the upslope areas computed for plateaus. 
% * New function: |postprocess_plateaus|.  This function flattens the upslope
%   areas computed for plateaus.  This function was formerly a part of
%   |upslope_area|. 
% 
%% Version 1.0, 02-Aug-2007
% Initial release
%
% Copyright 2007-2009 The MathWorks, Inc.