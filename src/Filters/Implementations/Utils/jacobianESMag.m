
% See understandJacobians.m for more informations

% coordinateSystem : ned or enu
function H = jacobianESMag(q, v1, vz, coordinateSystem)
	
	qw = q(1); qx = q(2); qy = q(3); qz = q(4);

	switch coordinateSystem
		case 'ned'
			vx = v1;
			H = [          -2*qy*vz,           2*qz*vz, - 2*qw*vz - 4*qy*vx, 2*qx*vz - 4*qz*vx
				  2*qx*vz - 2*qz*vx, 2*qw*vz + 2*qy*vx,   2*qx*vx + 2*qz*vz, 2*qy*vz - 2*qw*vx
				            2*qy*vx, 2*qz*vx - 4*qx*vz,   2*qw*vx - 4*qy*vz,           2*qx*vx];
		case 'enu'
			vy = v1;
			H = [ 2*qz*vy - 2*qy*vz,   2*qy*vy + 2*qz*vz, 2*qx*vy - 2*qw*vz, 2*qw*vy + 2*qx*vz
				            2*qx*vz,   2*qw*vz - 4*qx*vy,           2*qz*vz, 2*qy*vz - 4*qz*vy
				           -2*qx*vy, - 2*qw*vy - 4*qx*vz, 2*qz*vy - 4*qy*vz,           2*qy*vy];
		otherwise
			error('Unknown coordinateSystem')
	end

end
