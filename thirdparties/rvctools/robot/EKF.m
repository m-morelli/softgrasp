%EKF Extended Kalman Filter for vehicle pose and map estimation
%
% This class can be used for:
%   - dead reckoning localization
%   - map-based localization
%   - map making
%   - simultaneous localization and mapping
%
% It is used in conjunction with:
%   - a kinematic vehicle model that provides odometry output, represented 
%     by a Vehicle object.  
%   - The vehicle must be driven within the area of the map and this is 
%     achieved by connecting it to a Driver object.  
%   - a map containing the position of a number of landmarks, a Map object
%   - a sensor that returns measurements about landmarks relative to the 
%     vehicle's location.
%
% The EKF object updates its state at each time step, and invokes the 
% state update methods of the Vehicle.  The complete history of estimated
% state and covariance is stored within the EKF object.
%
% Methods::
%   run            run the filter
%   plot_xy        return/plot the actual path of the vehicle
%   plot_P         return/plot the estimate covariance 
%   plot_map       plot feature points and confidence limits
%   plot_ellipse   plot path with covariance ellipses
%   display        print the filter state in human readable form
%   char           convert the filter state to human readable string
%
% Properties::
%  x_est      estimated state
%  P          estimated covariance
%  V_est      estimated odometry covariance
%  W_est      estimated sensor covariance
%  features   map book keeping, maps sensor feature id to filter state
%  robot      reference to the robot object
%  sensor     reference to the sensor object
%  history    vector of structs that hold the detailed information from
%             each time step
%
% Vehicle position estimation::
%
% Create a vehicle with odometry covariance V, add a driver to it,
% create a Kalman filter with estimated covariance V_est and initial
% state covariance P0, then run the filter for N time steps.
%    veh = Vehicle(V);
%    veh.add_driver( RandomPath(20, 2) );
%    ekf = EKF(veh, V_est, P0);
%    ekf.run(N);
%
%
% Vehicle map based localization::
%
% Create a vehicle with odometry covariance V, add a driver to it,
% create a map with 20 point features, create a sensor that uses the map 
% and vehicle state to estimate feature range and bearing with covariance
% W, the Kalman filter with estimated covariances V_est and W_est and initial
% vehicle state covariance P0, then run the filter for N time steps.
%
%    veh = Vehicle(V);
%    veh.add_driver( RandomPath(20, 2) );
%    map = Map(20);
%    sensor = RangeBearingSensor(veh, map, W);
%    ekf = EKF(veh, V_est, P0, sensor, W_est, map);
%    ekf.run(N);
%
% Vehicle-based map making::
%
% Create a vehicle with odometry covariance V, add a driver to it,
% create a sensor that uses the map and vehicle state to estimate feature range 
% and bearing with covariance W, the Kalman filter with estimated sensor 
% covariance W_est and a "perfect" vehicle (no covariance),
% then run the filter for N time steps.
%
%    veh = Vehicle(V);
%    veh.add_driver( RandomPath(20, 2) );
%    sensor = RangeBearingSensor(veh, map, W);
%    ekf = EKF(veh, [], [], sensor, W_est, []);
%    ekf.run(N);
%
% Simultaneous localization and mapping (SLAM)::
%
% Create a vehicle with odometry covariance V, add a driver to it,
% create a map with 20 point features, create a sensor that uses the map 
% and vehicle state to estimate feature range and bearing with covariance
% W, the Kalman filter with estimated covariances V_est and W_est and initial
% state covariance P0, then run the filter for N time steps to estimate
% the vehicle state at each time step and the map.%    veh = Vehicle(V);
%
%    veh.add_driver( RandomPath(20, 2) );
%    map = Map(20);
%    sensor = RangeBearingSensor(veh, map, W);
%    ekf = EKF(veh, V_est, P0, sensor, W, []);
%    ekf.run(N);
%
% Reference::
%
%   Robotics, Vision & Control,
%   Peter Corke,
%   Springer 2011
%
% See also Vehicle, RandomPath, RangeBearingSensor, Map, ParticleFilter.

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
classdef EKF < handle

    %TODO
    % add a hook for data association
    properties
        xVehicle        % vehicle state


        % features keeps track of features we've seen before
        % each column represents a feature.  This is a fixed size
        % array, indexed by feature id.
        % row 1: the start of this feature's state in the feature
        %        part of the state vector, initially NaN
        % row 2: the number of times we've sighted the feature
        features           % map state

        verbose
        V_est           % estimate of covariance V
        W_est           % estimate of covariance W

        x_est           % estimated state
        P_est           % estimated covariance

        robot           % reference to the robot vehicle
        sensor          % reference to the sensor
        %map
        %   estVehicle    estMap
        %        0          0     
        %        0          1     make map
        %        1          0     dead reckoning
        %        1          1     SLAM

        estVehicle      % flag: estimating vehicle location
        estMap          % flag: estimating map

        joseph          % flag: use Joseph form to compute p

        history         % vector of structs to hold EKF history
    end

    methods

        % constructor
        function ekf = EKF(robot, V_est, P0, sensor, W_est, map)
            %EKF.EKF EKF object constructor
            %
            % E = EKF(VEHICLE, VEST, P0) is an EKF that estimates the state of
            % the VEHICLE with estimated odometry covariance VEST (2x2) and
            % initial covariance (3x3).
            %
            % E = EKF(VEHICLE, VEST, P0, SENSOR, WEST, MAP) as above but
            % uses information from a VEHICLE mounted sensor, estimated
            % sensor covariance WEST and a MAP.
            %
            % If MAP is [] then it will be estimated.
            %
            % If VEST and P0 are [] the vehicle is assumed error free and
            % the filter will estimate the landmark positions (map).
            %
            % If VEST and P0 are finite the filter will estimate the
            % vehicle pose and the landmark positions (map).
            %
            % Notes::
            % - EKF subclasses Handle, so it is a reference object.
            %
            % See also Vehicle, Sensor, RangeBearingSensor, Map.

            
            ekf.sensor = [];
            ekf.robot = robot;
            ekf.V_est = V_est
            if nargin > 3
                ekf.sensor = sensor;
                ekf.W_est = W_est;
            end
            joseph = true;
            if isempty(V_est)
                % perfect vehicle case
                ekf.estVehicle = false;
                ekf.x_est = [];
                ekf.P_est = [];
            else
                % noisy odometry case
                ekf.x_est = robot.x0;
                ekf.P_est = P0;
                ekf.estVehicle = true;
            end
            if nargin >= 6 && isempty(map)
                ekf.estMap = true;
                ekf.features = NaN*zeros(2, sensor.map.nfeatures);
            else
                ekf.estMap = false;
            end

            ekf.history = [];
        end

        function init(ekf)
            ekf.robot.init();

            % init the state vector
            if ekf.estVehicle
                ekf.x_est = ekf.robot.x;
            else
                ekf.x_est = [];
            end
            % clear the history
            ekf.history = [];
        end

        function run(ekf, n)
            %EKF.run Run the EKF
            %
            % E.run(N) run the filter for N time steps.
            %
            % Notes::
            % - all previously estimated states and estimation history is
            %   cleared.
            ekf.robot.init();

            for k=1:n
                ekf.step();
            end
        end


        function x_est = step(ekf)

            %fprintf('-------step\n');
            % move the robot along its path and get odometry
            odo = ekf.robot.step();

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %% do the prediction
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


            if ekf.estVehicle
                % split the state vector and covariance into chunks for 
                % vehicle and map
                xv_est = ekf.x_est(1:3);
                xm_est = ekf.x_est(4:end);

                Pvv_est = ekf.P_est(1:3,1:3);
                Pmm_est = ekf.P_est(4:end,4:end);
                Pvm_est = ekf.P_est(1:3,4:end);
            else
                xm_est = ekf.x_est;
                %Pvv_est = ekf.P_est;
                Pmm_est = ekf.P_est;
            end

            if ekf.estVehicle
                % evaluate the state update function and the Jacobians
                % if vehicle has uncertainty, predict its covariance
                xv_pred = ekf.robot.f(xv_est, odo);

                Fx = ekf.robot.Fx(xv_est, odo);
                Fv = ekf.robot.Fv(xv_est, odo);
                Pvv_pred = Fx*Pvv_est*Fx' + Fv*ekf.V_est*Fv';
             else
                % otherwise we just take the true robot state
                xv_pred = ekf.robot.x;
             end

            if ekf.estMap
                if ekf.estVehicle
                    % SLAM case, compute the correlations
                    Pvm_pred = Fx*Pvm_est;
                end
                Pmm_pred = Pmm_est;
                xm_pred = xm_est;
            end

            % put the chunks back together again
            if ekf.estVehicle && ~ekf.estMap
                % vehicle only
                x_pred = xv_pred;
                P_pred =  Pvv_pred;
            elseif ~ekf.estVehicle && ekf.estMap
                % map only
                x_pred = xm_pred;
                P_pred = Pmm_pred;
            elseif ekf.estVehicle && ekf.estMap
                % vehicle and map
                x_pred = [xv_pred; xm_pred];
                P_pred = [ Pvv_pred Pvm_pred; Pvm_pred' Pmm_pred];
            end

            % at this point we have:
            %   xv_pred the state of the vehicle to use to 
            %           predict observations
            %   xm_pred the state of the map
            %   x_pred  the full predicted state vector
            %   P_pred  the full predicted covariance matrix

            % initialize the variables that might be computed during
            % the update phase

            doUpdatePhase = false;

            %fprintf('x_pred:'); x_pred'

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %% process observations
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            sensorReading = false;
            if ~isempty(ekf.sensor)
                % read the sensor
                [z,js] = ekf.sensor.reading();
                % if isnan(i) then the sensor has not returned a reading
                % at this time interval
                sensorReading = ~isnan(js);
            end

            if sensorReading
                % here for MBL, MM, SLAM

                % compute the innovation
                z_pred = ekf.sensor.h(xv_pred, js);
                innov(1) = z(1) - z_pred(1);
                innov(2) = angdiff(z(2), z_pred(2));

                if ekf.estMap
                    % the map is estimated MM or SLAM case
                    if ekf.seenBefore(js)

                        % get previous estimate of its state
                        jx = ekf.features(1,js);
                        xf = xm_pred(jx:jx+1);

                        % compute Jacobian for this particular feature
                        Hx_k = ekf.sensor.Hxf(xv_pred, xf);
                        % create the Jacobian for all features
                        Hx = zeros(2, length(xm_pred));
                        Hx(:,jx:jx+1) = Hx_k;

                        Hw = ekf.sensor.Hw(xv_pred, xf);

                        if ekf.estVehicle
                            % concatenate Hx for for vehicle and map
                            Hxv = ekf.sensor.Hx(xv_pred, xf);
                            Hx = [Hxv Hx];
                        end
                        doUpdatePhase = true;

        %                if mod(i, 40) == 0
        %                    plot_ellipse(x_est(jx:jx+1), P_est(jx:jx+1,jx:jx+1), 5);
        %                end
                    else
                        % get the extended state
                        [x_pred, P_pred] = ekf.extendMap(P_pred, xv_pred, xm_pred, z, js);
                        doUpdatePhase = false;
                    end
                else
                    % the map is given, MBL case
                    Hx = ekf.sensor.Hx(xv_pred, js);
                    Hw = ekf.sensor.Hw(xv_pred, js);
                    doUpdatePhase = true;
                end
            end

                % doUpdatePhase flag indicates whether or not to do
                % the update phase of the filter
                %
                %  DR                        always false
                %  map-based localization    if sensor reading
                %  map creation              if sensor reading & not first
                %                              sighting
                %  SLAM                      if sighting of a previously
                %                              seen feature

            if doUpdatePhase
                %fprintf('do update\n');
                %% we have innovation, update state and covariance
                % compute x_est and P_est

                % compute innovation covariance
                S = Hx*P_pred*Hx' + Hw*ekf.W_est*Hw';

                % compute the Kalman gain
                K = P_pred*Hx'*inv(S);

                % update the state vector
                x_est = x_pred + K*innov';
            
                if ekf.estVehicle
                    % wrap heading state for a vehicle
                    x_est(3) = angdiff(x_est(3));
                end
            
                % update the covariance
                if ekf.joseph
                    % we use the Joseph form
                    I = eye(size(P_est));
                    P_est = (I-K*Hx)*P_pred*(I-K*Hx)' + K*ekf.W_est*K';
                else
                    P_est = P_pred - K*S*K';
                end

                % enforce P to be symmetric
                P_est = 0.5*(P_est+P_est');
            else
                % no update phase, estimate is same as prediction
                x_est = x_pred;
                P_est = P_pred;
                innov = [];
                S = [];
                K = [];
            end

            %fprintf('X:'); x_est'

            % update the state and covariance for next time
            ekf.x_est = x_est;
            ekf.P_est = P_est;

            % record time history
            hist = [];
            hist.x_est = x_est;
            hist.odo = odo;
            hist.P = P_est;
            hist.innov = innov;
            hist.S = S;
            hist.K = K;
            ekf.history = [ekf.history hist];
        end

        function s = seenBefore(ekf, jf)
            if ~isnan(ekf.features(1,jf))
                %% we have seen this feature before, update number of sightings
                if ekf.verbose
                    fprintf('feature %d seen %d times before, state_idx=%d\n', ...
                        jf, ekf.features(2,jf), ekf.features(1,jf));
                end
                ekf.features(2,jf) = ekf.features(2,jf)+1;  
                s = true;
            else
                s = false;
            end
        end
                

        function [x_ext, P_ext] = extendMap(ekf, P, xv, xm, z, jf)
            
            %% this is a new feature, we haven't seen it before
            % estimate position of feature in the world based on 
            % noisy sensor reading and current vehicle pose

            if ekf.verbose
                fprintf('feature %d first sighted\n', jf);
            end

            % estimate its position based on observation and vehicle state
            xf = ekf.sensor.g(xv, z);

            % append this estimate to the state vector
            if ekf.estVehicle
                x_ext = [xv; xm; xf];
            else
                x_ext = [xm; xf];
            end

            % get the Jacobian for the new feature
            Gz = ekf.sensor.Gz(xv, z);

            % extend the covariance matrix
            if ekf.estVehicle
                Gx = ekf.sensor.Gx(xv, z);
                n = length(ekf.x_est);
                M = [eye(n) zeros(n,2); Gx zeros(2,n-3) Gz];
                P_ext = M*blkdiag(P, ekf.W_est)*M';
            else
                P_ext = blkdiag(P, Gz*ekf.W_est*Gz');
            end
    
            % record the position in the state vector where this 
            % feature's state starts
            ekf.features(1,jf) = length(xm)+1;
            %ekf.features(1,jf) = length(ekf.x_est)-1;
            ekf.features(2,jf) = 1;        % seen it once

            if ekf.verbose
                fprintf('extended state vector\n');
            end

            % plot an ellipse at this time
%                jx = features(1,jf);
%                plot_ellipse(x_est(jx:jx+1), P_est(jx:jx+1,jx:jx+1), 5);

        end

        function verbosity(ekf, v)
            ekf.verbose = v;
        end
            
        function out = plot_xy(ekf, varargin)
            %EKF.plot_xy Plot vehicle position
            %
            % E.plot_xy() plot the estimated vehicle path in the xy-plane.
            %
            % E.plot_xy(LS) as above but the optional line style arguments
            % LS are passed to plot.
            if ekf.estVehicle
                xyt = [];
                for h=ekf.history
                    xyt = [xyt; h.x_est(1:3)'];
                end
                if nargout == 0
                    plot(xyt(:,1), xyt(:,2), varargin{:});
                end
            else
                xyt = [];
            end
            if nargout > 0
                out = xyt;
            end
        end
        
        % TODO:  some option to plot map evolution, layered ellipses
        function out = plot_map(ekf, interval, varargin)
            %EKF.plot_map Plot landmarks
            %
            % E.plot_map(I) overlay the current plot with the estimated
            % landmark position (a +-marker) and a covariance ellipses for every I'th time
            % step.
            %
            % E.plot_map() as above but I=20.
            %
            % E.plot_map(I, LS) as above but pass line style arguments
            % LS to plot_ellipse.
            %
            % See also plot_ellipse.
            if nargin < 2 || isempty(interval)
                interval = round(length(ekf.history)/20);
            end
            for i=1:numcols(ekf.features)
                n = ekf.features(1,i);
                if isnan(n)
                    continue;
                end
                % n is an index into the *feature* part of the state
                % vector, we need to offset it to account for the vehicle
                % state if we are estimating vehicle as well
                if ekf.estVehicle
                    n = n + 3;
                end
                xf = ekf.x_est(n:n+1);
                P = ekf.P_est(n:n+1,n:n+1);
                % TODO reinstate the interval feature
                %plot_ellipse(xf, P, interval, 0, [], varargin{:});
                plot_ellipse(P, xf, varargin{:});
                plot(xf(1), xf(2), '+')
            end
        end

        function out = plot_P(ekf, varargin)
            %EKF.plot_P Plot covariance magnitude
            %
            % E.plot_P() plots the estimated covariance magnitude against
            % time step.
            %
            % E.plot_P(LS) as above but the optional line style arguments
            % LS are passed to plot.
            %
            % M = E.plot_P() returns the estimated covariance magnitude at
            % all time steps as a vector.
            p = [];
            for h=ekf.history
                p = [p; sqrt(det(h.P))];
            end
             if nargout == 0
                 plot(p, varargin{:});
                 xlabel('sample');
                 ylabel('(det P)^{0.5}')
             else
                out = p;
             end
        end

        function plot_ellipse(ekf, interval, varargin)
            %EKF.plot_ellipse Plot vehicle covariance as an ellipse
            %
            % E.plot_ellipse(I) overlay the current plot with the estimated
            % vehicle position covariance ellipses for every I'th time
            % step.
            %
            % E.plot_ellipse() as above but I=20.
            %
            % E.plot_ellipse(I, LS) as above but pass line style arguments
            % LS to plot_ellipse.
            %
            % See also plot_ellipse.

            if nargin < 2 || isempty(interval)
                interval = round(length(ekf.history)/20);
            end
            holdon = ishold;
            hold on
            for i=1:interval:length(ekf.history)
                h = ekf.history(i);
                %plot_ellipse(h.x_est(1:2), h.P(1:2,1:2), 1, 0, [], varargin{:});
                plot_ellipse(h.P(1:2,1:2), h.x_est(1:2), varargin{:});
            end
            if ~holdon
                hold off
            end
        end
            
        function display(ekf)
            %EKF.display Display status of EKF object
            %
            % E.display() display the state of the EKF object in
            % human-readable form.
            %
            % Notes::
            % - this method is invoked implicitly at the command line when the result
            %   of an expression is a EKF object and the command has no trailing
            %   semicolon.
            %
            % See also EKF.char.
            
            loose = strcmp( get(0, 'FormatSpacing'), 'loose');
            if loose
                disp(' ');
            end
            disp([inputname(1), ' = '])
            disp( char(ekf) );
        end % display()

        function s = char(ekf)
            %EKF.char Convert EKF object to string
            %
            % E.char() is a string representing the state of the EKF
            % object in human-readable form.
            s = sprintf('EKF object: %d states', length(ekf.x_est));
            e = '';
            if ekf.estVehicle
                e = [e 'Vehicle '];
            end
            if ekf.estMap
                e = [e 'Map '];
            end
            s = strvcat(s, ['  estimating: ' e]);
        end


    end % method
end % classdef



 
