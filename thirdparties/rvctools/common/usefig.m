%USEFIG	reuse a named figure or create a new figure
%
%	usefig('Foo')
%
%	make figure 'Foo' the current figure, if it doesn't exist create it.
%
%	h = usefig('Foo') as above, but returns the figure handle

function H = usefig(name)

	h = findobj('Name', name);
	if isempty(h),
		h = figure;
		set(h, 'Name', name);
	else
		figure(h);
	end

	if nargout > 0,
		H = h;
	end
