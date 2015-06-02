%% Functions by Category
% Upslope Area Toolbox
% Version 2.0 09-Dec-2009
%
% Requires Image Processing Toolbox(TM).
%
%% Flow Direction
% * <demFlow_help.html |demFlow|> - Downslope flow direction for a DEM
% * <facetFlow_help.html |facetFlow|> - Facet flow direction
% * <flowMatrix_help.html |flowMatrix|> - Linear equations representing water flow
% * <pixelFlow_help.html |pixelFlow|>           - Downslope flow direction for DEM pixels
%
%% Preprocessing and Postprocessing
% * <borderNans_help.html |borderNans|> - Find NaNs connected to DEM border
% * <fillSinks_help.html |fillSinks|> - Fill interior sinks in a DEM
% * <postprocessPlateaus_help.html |postprocessPlateaus|> - Replace upslope areas for plateaus with mean value 
%
%% Hydrological Applications
% * <dependenceMap_help.html |dependenceMap|> - Dependence map for water flow in a DEM
% * <influenceMap_help.html |influenceMap|> - Influence map for water flow in a DEM
% * <upslopeArea_help.html |upslopeArea|> - Upslope area measurements for a DEM
%
%% Display
% * <visDemFlow_help.html |visDemFlow|> - Visualize flow directions in a DEM
% * <visMap_help.html |visMap|> - Visualize influence or dependence map for a DEM
%
%% Data
% * milford_ma_dem.mat - Sample DEM data provided by USGS and distributed
% via Geo Community (geoworld.com), a USGS data
% distribution partner.  The data set is a 1:24,000-scale
% raster profile digital elevation model.  Download the
% "Milford" file from the "Digital Elevation Models (DEM)
% - 24K Middlesex County, Massachusetts, United States"
% page at http://data.geocomm.com/catalog/US/61059/526/group4-3.html.
% * natick_ned* - Sample 1/3 arc-second DEM data for a region in Natick,
% Massachusetts.  Downloaded from the The National Map Seamless Server
% (http://seamless.usgs.gov/index.php).
%
%% Source
% Steven L. Eddins
% Copyright 2007-2009 The MathWorks, Inc.
