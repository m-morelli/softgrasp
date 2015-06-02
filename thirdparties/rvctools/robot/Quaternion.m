%QUATERNION Quaternion class
% 
% A quaternion is a compact method of representing a 3D rotation that has
% computational advantages including speed and numerical robustness.
% A quaternion has 2 parts, a scalar s, and a vector v and is typically 
% written: q = s <vx, vy, vz>.  
%
% A unit quaternion is one for which s^2+vx^2+vy^2+vz^2 = 1.  It can be 
% considered as a rotation about a vector in space where 
% q = cos (theta/2) < v sin(theta/2)> where v is a unit vector.
%
% Q = Quaternion(X) is a unit quaternion equivalent to X which can be any
% of:
%   - orthonormal rotation matrix.
%   - homogeneous transformation matrix (rotation part only).
%   - rotation angle and vector
%
% Methods::
%  inv       return inverse of quaterion
%  norm      return norm of quaternion
%  unit      return unit quaternion
%  unitize   unitize this quaternion
%  plot      same options as trplot()
%  interp    interpolation (slerp) between q and q2, 0<=s<=1
%  scale     interpolation (slerp) between identity and q, 0<=s<=1
%  dot       derivative of quaternion with angular velocity w
%
% Arithmetic operators are overloaded::
%  q+q2      return elementwise sum of quaternions
%  q-q2      return elementwise difference of quaternions
%  q*q2      return quaternion product
%  q*v       rotate vector by quaternion, v is 3x1
%  q/q2      return q*q2.inv
%  q^n       return q to power n (integer only)
%
% Properties (read only)::
%  s         real part
%  v         vector part
%  R         3x3 rotation matrix
%  T         4x4 homogeneous transform matrix
%
% Notes::
% - Quaternion objects can be used in vectors and arrays
%
% See also trinterp, trplot.

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
%  constructor handles R, T trajectory and returns vector
%  .r, .t on a quaternion vector??

classdef Quaternion

    properties (Dependent = true)
        R   % rotation matrix
        T   % translation matrix
    end

    properties (SetAccess = private)
        s       % scalar part
        v       % vector part
    end

    methods

        function q = Quaternion(a1, a2)
        %Quaternion.Quaternion Constructor for quaternion objects
        % 
        % Q = Quaternion() is the identitity quaternion 1<0,0,0> representing a null rotation.
        %
        % Q = Quaternion(Q1) is a copy of the quaternion Q1
        %
        % Q = Quaternion([S V1 V2 V3]) is a quaternion formed by specifying directly its 4 elements
        %
        % Q = Quaternion(S) is a quaternion formed from the scalar S and zero vector part: S<0,0,0>
        %
        % Q = Quaternion(V) is a pure quaternion with the specified vector part: 0<V>
        %
        % Q = Quaternion(TH, V) is a unit quaternion corresponding to rotation of TH about the vector V.
        %
        % Q = Quaternion(R) is a unit quaternion corresponding to the orthonormal rotation matrix R.
        %
        % Q = Quaternion(T) is a unit quaternion equivalent to the rotational
        % part of the homogeneous transform T.

            if nargin == 0
                q.v = [0,0,0];
                q.s = 1;
            elseif isa(a1, 'Quaternion')
            %   Q = Quaternion(q)       from another quaternion
                q = a1;
            elseif nargin == 1
                if all(size(a1) == [1 4]) || all(size(a1) == [4 1])
            %   Q = Quaternion([s v1 v2 v3])    from 4 elements
                    a1 = a1(:);
                    q.s = a1(1);
                    q.v = a1(2:4)';
                elseif all(size(a1) == [3 3])
            %   Q = Quaternion(R)       from a 3x3 or 4x4 matrix
                    q = Quaternion( tr2q(a1) );
                elseif all(size(a1) == [4 4])
                    q = Quaternion( tr2q(a1(1:3,1:3)) );

                elseif length(a1) == 3
            %   Q = Quaternion(v)       from a vector

                    q.s = 0;
                    q.v = a1(:)';
                elseif length(a1) == 1
            %   Q = Quaternion(s)       from a scalar
                    q.s = a1(1);
                    q.v = [0 0 0];
                else
                    error('unknown dimension of input');
                end
            elseif nargin == 2
                if isvector(a2) && isscalar(a1)
                %   Q = Quaternion(theta, v)    from vector plus angle
                    q.s = cos(a1/2);
                    q.v = sin(a1/2)*unit(a2(:)');
                end
            end

        end


        function s = char(q)
        %Quaternion.char Create string representation of quaternion object
        %
        % S = Q.char() is a compact string representation of the quaternion's value
        % as a 4-tuple.

            if length(q) > 1
                s = '';
                for qq = q;
                    s = strvcat(s, char(qq));
                end
                return
            end
            s = [num2str(q.s), ' < ' ...
                num2str(q.v(1)) ', ' num2str(q.v(2)) ', '   num2str(q.v(3)) ' >'];
        end


        function display(q)
        %Quaternion.display Display the value of a quaternion object
        %
        % Q.display() displays a compact string representation of the quaternion's value
        % as a 4-tuple.
        %
        % Notes::
        % - this method is invoked implicitly at the command line when the result
        %   of an expression is a Quaternion object and the command has no trailing
        %   semicolon.
        %
        % See also Quaternion.char.


            loose = strcmp( get(0, 'FormatSpacing'), 'loose');
            if loose
                disp(' ');
            end
            disp([inputname(1), ' = '])
            if loose
                disp(' ');
            end
            disp(char(q))
            if loose
                disp(' ');
            end
        end


        function v = double(q)
        %Quaternion.double Convert a quaternion object to a 4-element vector
        %
        % V = Q.double() is a 4-vector comprising the quaternion
        % elements [s vx vy vz].

            v = [q.s q.v];
        end

        function qi = inv(q)
        %Quaternion.inv Invert a unit-quaternion
        %
        % QI = Q.inv() is a quaternion object representing the inverse of Q.

            qi = Quaternion([q.s -q.v]);
        end

        function qu = unit(q)
        %Quaternion.unit Unitize a quaternion
        %
        % QU = Q.unit() is a quaternion which is a unitized version of  Q

            qu = q / norm(q);
        end

        function n = norm(q)
        %Quaternion.norm Compute the norm of a quaternion
        %
        % QN = Q.norm(Q) is the scalar norm or magnitude of the quaternion Q.

            n = norm(double(q));
        end


        function q = interp(Q1, Q2, r)
        %Quaternion.interp Interpolate rotations expressed by quaternion objects
        %
        % QI = Q1.interp(Q2, R) is a unit-quaternion that interpolates between Q1 for R=0 
        % to Q2 for R=1. This is a spherical linear interpolation (slerp) that can be 
        % interpretted as interpolation along a great circle arc on a sphere.
        %
        % If R is a vector QI is a vector of quaternions, each element
        % corresponding to sequential elements of R.
        %
        % Notes:
        % - the value of r is clipped to the interval 0 to 1
        %
        % See also ctraj, Quaternion.scale.

            q1 = double(Q1);
            q2 = double(Q2);

            theta = acos(q1*q2');
            count = 1;

            % clip values of r
            r(r<0) = 0;
            r(r>1) = 1;
            
            if length(r) == 1
                if theta == 0
                    q = Q1;
                else
                    q = Quaternion( (sin((1-r)*theta) * q1 + sin(r*theta) * q2) / sin(theta) );
                end
            else
                for R=r(:)'
                    if theta == 0
                        qq = Q1;
                    else
                        qq = Quaternion( (sin((1-R)*theta) * q1 + sin(R*theta) * q2) / sin(theta) );
                    end
                    q(count) = qq;
                    count = count + 1;
                end
            end
        end
        
        
        function q = scale(Q, r)
        %Quaternion.scale Interpolate rotations expressed by quaternion objects
        %
        % QI = Q.scale(R) is a unit-quaternion that interpolates between identity for R=0
        % to Q for R=1.  This is a spherical linear interpolation (slerp) that can
        % be interpretted as interpolation along a great circle arc on a sphere.
        %
        % If R is a vector QI is a cell array of quaternions, each element
        % corresponding to sequential elements of R.
        %
        % See also ctraj, Quaternion.interp.


            q2 = double(Q);

            if any(r<0) || (r>1)
                error('r out of range');
            end
            q1 = [1 0 0 0];         % identity quaternion
            theta = acos(q1*q2');

            if length(r) == 1
                if theta == 0
                    q = Q;
                else
                    q = Quaternion( (sin((1-r)*theta) * q1 + sin(r*theta) * q2) / sin(theta) ).unit;
                end
            else
                count = 1;
                for R=r(:)'
                    if theta == 0
                        qq = Q;
                    else
                        qq = Quaternion( (sin((1-r)*theta) * q1 + sin(r*theta) * q2) / sin(theta) ).unit;
                    end
                    q(count) = qq;
                    count = count + 1;
                end
            end
        end


        function qp = plus(q1, q2)
        %PLUS Add two quaternion objects
        %
        % Q1+Q2 is the element-wise sum of quaternion elements.

            if isa(q1, 'Quaternion') & isa(q2, 'Quaternion')

                qp = Quaternion(double(q1) + double(q2));
            end
        end


        function qp = minus(q1, q2)
        %Quaternion.minus Subtract two quaternion objects
        %
        % Q1-Q2 is the element-wise difference of quaternion elements.

            if isa(q1, 'Quaternion') & isa(q2, 'Quaternion')

                qp = Quaternion(double(q1) - double(q2));
            end
        end

        function qp = mtimes(q1, q2)
        %Quaternion.mtimes Multiply a quaternion object
        %
        % Q1*Q2   is a quaternion formed by Hamilton product of two quaternions.
        % Q*V     is the vector V rotated by the quaternion Q
        % Q*S     is the element-wise multiplication of quaternion elements by by the scalar S

            if isa(q1, 'Quaternion') & isa(q2, 'Quaternion')
            %QQMUL  Multiply unit-quaternion by unit-quaternion
            %
            %   QQ = qqmul(Q1, Q2)
            %
            %   Return a product of unit-quaternions.
            %
            %   See also: TR2Q


                % decompose into scalar and vector components
                s1 = q1.s;  v1 = q1.v;
                s2 = q2.s;  v2 = q2.v;

                % form the product
                qp = Quaternion([s1*s2-v1*v2' s1*v2+s2*v1+cross(v1,v2)]);

            elseif isa(q1, 'Quaternion') & isa(q2, 'double')

            %QVMUL  Multiply vector by unit-quaternion
            %
            %   VT = qvmul(Q, V)
            %
            %   Rotate the vector V by the unit-quaternion Q.
            %
            %   See also: QQMUL, QINV

                if length(q2) == 3
                    qp = q1 * Quaternion([0 q2(:)']) * inv(q1);
                    qp = qp.v(:);
                elseif length(q2) == 1
                    qp = Quaternion( double(q1)*q2);
                else
                    error('quaternion-vector product: must be a 3-vector or scalar');
                end

            elseif isa(q2, 'Quaternion') & isa(q1, 'double')
                if length(q1) == 3
                    qp = q2 * Quaternion([0 q1(:)']) * inv(q2);
                    qp = qp.v;
                elseif length(q1) == 1
                    qp = Quaternion( double(q2)*q1);
                else
                    error('quaternion-vector product: must be a 3-vector or scalar');
                end
            end
        end

        function qp = mpower(q, p)
        %Quaternion.mpower Raise quaternion to integer power
        %
        % Q^N is quaternion Q raised to the integer power N, and computed by repeated multiplication.

            % check that exponent is an integer
            if (p - floor(p)) ~= 0
                error('quaternion exponent must be integer');
            end

            qp = q;

            % multiply by itself so many times
            for i = 2:abs(p)
                qp = qp * q;
            end

            % if exponent was negative, invert it
            if p<0
                qp = inv(qp);
            end
        end

        function qq = mrdivide(q1, q2)
        %Quaternion.mrdivide Compute quaternion quotient.
        %
        % Q1/Q2   is a quaternion formed by Hamilton product of Q1 and inv(Q2)
        % Q/S     is the element-wise division of quaternion elements by by the scalar S

            if isa(q2, 'Quaternion')
                % qq = q1 / q2
                %    = q1 * qinv(q2)

                qq = q1 * inv(q2);
            elseif isa(q2, 'double')
                qq = Quaternion( double(q1) / q2 );
            end
        end


        function plot(Q, varargin)
        %Quaternion.plot Plot a quaternion object 
        %
        % Q.plot(options) plots the quaternion as a rotated coordinate frame.
        %
        % See also trplot.

            %axis([-1 1 -1 1 -1 1])

            if nargin < 2
                off = [0 0 0];
            end
            if nargin < 3
                fmt = '%c';
            end
            if nargin < 4
                color = 'b';
            end

            trplot( Q.R, varargin{:});
            drawnow
        end

        function r = get.R(q)
        %Quaternion.R Return equivalent orthonormal rotation matrix
        %
        % R = Q.R is the equivalent 3x3 orthonormal rotation matrix.
            r = t2r( q2tr(q) );
        end

        function t = get.T(q)
        %Quaternion.T Return equivalent homogeneous transformationmatrix
        %
        % T = Q.T is the equivalent 4x4 homogeneous transformation matrix.
            t = q2tr(q);
        end

        function qd = dot(q, omega)
            E = q.s*eye(3,3) - skew(q.v);
            omega = omega(:);
            qd = Quaternion(-0.5*q.v*omega, 0.5*E*omega);
        end
    end % methods
end % classdef

%TR2Q   Convert homogeneous transform to a unit-quaternion
%
%   Q = tr2q(T)
%
%   Return a unit quaternion corresponding to the rotational part of the
%   homogeneous transform T.
%
%   See also: Q2TR

function q = tr2q(t)
    qs = sqrt(trace(t)+1)/2.0;
    kx = t(3,2) - t(2,3);   % Oz - Ay
    ky = t(1,3) - t(3,1);   % Ax - Nz
    kz = t(2,1) - t(1,2);   % Ny - Ox

    if (t(1,1) >= t(2,2)) & (t(1,1) >= t(3,3)) 
        kx1 = t(1,1) - t(2,2) - t(3,3) + 1; % Nx - Oy - Az + 1
        ky1 = t(2,1) + t(1,2);          % Ny + Ox
        kz1 = t(3,1) + t(1,3);          % Nz + Ax
        add = (kx >= 0);
    elseif (t(2,2) >= t(3,3))
        kx1 = t(2,1) + t(1,2);          % Ny + Ox
        ky1 = t(2,2) - t(1,1) - t(3,3) + 1; % Oy - Nx - Az + 1
        kz1 = t(3,2) + t(2,3);          % Oz + Ay
        add = (ky >= 0);
    else
        kx1 = t(3,1) + t(1,3);          % Nz + Ax
        ky1 = t(3,2) + t(2,3);          % Oz + Ay
        kz1 = t(3,3) - t(1,1) - t(2,2) + 1; % Az - Nx - Oy + 1
        add = (kz >= 0);
    end

    if add
        kx = kx + kx1;
        ky = ky + ky1;
        kz = kz + kz1;
    else
        kx = kx - kx1;
        ky = ky - ky1;
        kz = kz - kz1;
    end
    nm = norm([kx ky kz]);
    if nm == 0
        q = Quaternion([1 0 0 0]);
    else
        s = sqrt(1 - qs^2) / nm;
        qv = s*[kx ky kz];

        q = Quaternion([qs qv]);

    end
end


%Q2TR   Convert unit-quaternion to homogeneous transform
%
%   T = q2tr(Q)
%
%   Return the rotational homogeneous transform corresponding to the unit
%   quaternion Q.
%
%   See also: TR2Q

function t = q2tr(q)

    q = double(q);
    s = q(1);
    x = q(2);
    y = q(3);
    z = q(4);

    r = [   1-2*(y^2+z^2)   2*(x*y-s*z) 2*(x*z+s*y)
        2*(x*y+s*z) 1-2*(x^2+z^2)   2*(y*z-s*x)
        2*(x*z-s*y) 2*(y*z+s*x) 1-2*(x^2+y^2)   ];
    t = eye(4,4);
    t(1:3,1:3) = r;
    t(4,4) = 1;
end
