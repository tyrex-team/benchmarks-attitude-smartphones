function vo = quatrotate(q, v, form)

	qw = q(1); qx = q(2); qy = q(3); qz = q(4);
	vx = v(1); vy = v(2); vz = v(3);


	if ~exist('form', 'var') 
		form = 'short';
	end

	switch form

		case 'long'
			vo = [ 	vx*(qw^2 + qx^2 - qy^2 - qz^2) + vy*(2*qw*qz + 2*qx*qy) - vz*(2*qw*qy - 2*qx*qz) ...
					vy*(qw^2 - qx^2 + qy^2 - qz^2) - vx*(2*qw*qz - 2*qx*qy) + vz*(2*qw*qx + 2*qy*qz) ...
					vz*(qw^2 - qx^2 - qy^2 + qz^2) + vx*(2*qw*qy + 2*qx*qz) - vy*(2*qw*qx - 2*qy*qz)];

		case 'short'
			vo = [	vy*(2*qw*qz + 2*qx*qy) - vx*(2*qy^2 + 2*qz^2 - 1) 	- vz*(2*qw*qy - 2*qx*qz) 	...
				 	vz*(2*qw*qx + 2*qy*qz) - vx*(2*qw*qz - 2*qx*qy) 	- vy*(2*qx^2 + 2*qz^2 - 1) 	...
				 	vx*(2*qw*qy + 2*qx*qz) - vz*(2*qx^2 + 2*qy^2 - 1)	- vy*(2*qw*qx - 2*qy*qz)];

	 	otherwise
			error('Not a known form (short or long)');
	end
end
