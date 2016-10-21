
% See understandJacobians.m for more informations

function H = jacobianSE(q, v, form)

	qw = q(1); qx = q(2); qy = q(3); qz = q(4);
	vx = v(1); vy = v(2); vz = v(3);


	if ~exist('form', 'var'), form = 'short'; end

	switch form

		case 'long'
			
			H =	[ 	2*qw*vx + 2*qy*vz - 2*qz*vy, 2*qx*vx + 2*qy*vy + 2*qz*vz, 2*qw*vz + 2*qx*vy - 2*qy*vx, 2*qx*vz - 2*qw*vy - 2*qz*vx;
				 	2*qw*vy - 2*qx*vz + 2*qz*vx, 2*qy*vx - 2*qx*vy - 2*qw*vz, 2*qx*vx + 2*qy*vy + 2*qz*vz, 2*qw*vx + 2*qy*vz - 2*qz*vy;
				 	2*qw*vz + 2*qx*vy - 2*qy*vx, 2*qw*vy - 2*qx*vz + 2*qz*vx, 2*qz*vy - 2*qy*vz - 2*qw*vx, 2*qx*vx + 2*qy*vy + 2*qz*vz];

			% Equivalient
			% q1 = q(1);
			% u = q(2:4);
			% t = q1*x + skew(u) * x;
			% H = 2 * [t dot(u,x) * eye(3) - skew(t)];  

		case 'short'
			H = [ 	2*qy*vz - 2*qz*vy,           2*qy*vy + 2*qz*vz, 2*qw*vz + 2*qx*vy - 4*qy*vx, 2*qx*vz - 2*qw*vy - 4*qz*vx ;
					2*qz*vx - 2*qx*vz, 2*qy*vx - 4*qx*vy - 2*qw*vz,           2*qx*vx + 2*qz*vz, 2*qw*vx + 2*qy*vz - 4*qz*vy ;
					2*qx*vy - 2*qy*vx, 2*qw*vy - 4*qx*vz + 2*qz*vx, 2*qz*vy - 4*qy*vz - 2*qw*vx,           2*qx*vx + 2*qy*vy ];

		otherwise
			error('Not a known form (short or long)');
	end

end
