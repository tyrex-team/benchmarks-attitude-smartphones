% Shortcut for quatrotate(q, [0 0 1]);
function vo = quatrotateAcc(q, coordinateSystem)

	qw = q(1); qx = q(2); qy = q(3); qz = q(4);

	switch coordinateSystem
		case 'ned'
			vo = [   2*qw*qy - 2*qx*qz
					- 2*qw*qx - 2*qy*qz
					2*qx^2 + 2*qy^2 - 1];
		case 'enu'
			vo = [ 	 2*qx*qz - 2*qw*qy
				     2*qw*qx + 2*qy*qz
				 	 - 2*qx^2 - 2*qy^2 + 1];
		otherwise
			error('Unknown coordinateSystem')
	end

end
