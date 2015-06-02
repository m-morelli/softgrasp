%MDL_S4ABB2p8  Create kinematic model of ABB S4 2.8robot 
%
%	mdl_s4abb2P8
%
% Script creates the workspace variable R which describes the 
% kinematic characteristics of an ABB S4 2.8 robot using standard 
% DH conventions.
%
% Also define the workspace vector:
%   q0   mastering position.
%
% See also SerialLink, mdl_puma560akb, mdl_stanford, mdl_twolink.
%
% Author:
%  Wynand Swart,
%  Mega Robots CC, P/O Box 8412, Pretoria, 0001, South Africa
%  wynand.swart@gmail.com

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

%Cell: 073-1555-430
%30 Sep 2007
%S4 ABB 2.8 robot

%            theta    d      a    alpha
L(1) = link([ 0      0.9    0.188  -pi/2   0]);
L(2) = link([ 0      0      0.95    0      0]);
L(3) = link([ 0      0      0.225  -pi/2   0]);
L(4) = link([ 0      1.705  0       pi/2   0]);
L(5) = link([ 0      0      0      -pi/2   0]);
L(6) = link([ 0      0.2    0      -pi/2   0]);
%##########################################################
%Pose 0; At SYNCHRONISATION position
%##########################################################
q0 = [0     -pi/2         0       0      0     -pi/2];
R=robot(L, 'name', 'S4 ABB 2.8');
%##########################################################
