
% See understandJacobians.m for more informations

% coordinateSystem : ned or enu
function H = jacobianESAcc(q, coordinateSystem)

	qw = q(1); qx = q(2); qy = q(3); qz = q(4);

	switch coordinateSystem
		case 'ned'
			H = [  2*qy, -2*qz,  2*qw, -2*qx
				  -2*qx, -2*qw, -2*qz, -2*qy
				      0,  4*qx,  4*qy,     0];
		case 'enu'
			H = [  -2*qy,  2*qz, -2*qw, 2*qx
		   			2*qx,  2*qw,  2*qz, 2*qy
		      		   0, -4*qx, -4*qy,    0];
		otherwise
			error('Unknown coordinateSystem')
	end

end
