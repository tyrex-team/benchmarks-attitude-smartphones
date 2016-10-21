% Shortcut for quatrotate(q, [0 vy vz]) or quatrotate(q, [vx 0 vz])
function vo = quatrotateMag(q, v1, vz)

	qw = q(1); qx = q(2); qy = q(3); qz = q(4);

	switch coordinateSystem
		case 'ned'
			vx = v1;
			vo = [   - vx*(2*qy^2 + 2*qz^2 - 1) - vz*(2*qw*qy - 2*qx*qz)
					     vz*(2*qw*qx + 2*qy*qz) - vx*(2*qw*qz - 2*qx*qy)
					   vx*(2*qw*qy + 2*qx*qz) - vz*(2*qx^2 + 2*qy^2 - 1)];
		case 'enu'
			vy = v1;
			vo = [ 	 	 vy*(2*qw*qz + 2*qx*qy) - vz*(2*qw*qy - 2*qx*qz)
					   vz*(2*qw*qx + 2*qy*qz) - vy*(2*qx^2 + 2*qz^2 - 1)
					 - vz*(2*qx^2 + 2*qy^2 - 1) - vy*(2*qw*qx - 2*qy*qz)];
		otherwise
			error('Unknown coordinateSystem')
	end
end
