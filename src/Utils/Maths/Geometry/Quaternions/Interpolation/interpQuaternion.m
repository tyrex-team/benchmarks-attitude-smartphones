function vq = interpQuaternion(x, v, xq)

	N = length(x);
	Nq = length(xq);
	iq = 1;
	vq = [];

	for i = 2 : N

		qi = v(i-1, :);
		qn = v(i, :);

		tstart = x(i-1);
		tend = x(i);
		tdiff = tend - tstart;

		while iq <= Nq && (i == N || xq(iq) <= tend)
			t = (xq(iq) - tstart) / tdiff;
			vq(end+1, :) = slerp(qi, qn, t);
			iq = iq + 1;
		end

	end

end