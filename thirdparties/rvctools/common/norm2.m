%NORM2 columnwise norm
%
%	n = norm2(m)
%	n = norm2(a, b)

function n = norm2(a, b)

	if nargin == 1,
		n = sqrt( sum(a.^2) );
	elseif nargin == 2,
		n = sqrt( a(:).^2 + b(:).^2 );
	end
