% Convert a quaternion to a direct cosine matrix

function dcm = quat2dcm(q, form)

	if ~exist('form', 'var'), form = 'long'; end

	switch form

	case 'long'
		dcm = [ q(1)^2 + q(2)^2 - q(3)^2 - q(4)^2 		2*(q(2)*q(3) + q(1)*q(4)) 				2*(q(2)*q(4) - q(1)*q(3)) 
				2*(q(2)*q(3) - q(1)*q(4)) 				q(1)^2 - q(2)^2 + q(3)^2 - q(4)^2 		2*(q(3)*q(4) + q(1)*q(2))
				2*(q(2)*q(4) + q(1)*q(3)) 				2*(q(3)*q(4) - q(1)*q(2)) 				q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2];

		% Equivalient
		% dcm = (eye(3) + ...
		% 	2 * [-q(3)^2-q(4)^2 q(2)*q(3) q(2)*q(4);
		% 	q(2)*q(3)	-q(2)^2-q(4)^2 q(3)*q(4);
		% 	q(2)*q(4)	q(3)*q(4)	-q(2)^2-q(3)^2] + ...
		% 	2 * q(1) * skew(q(2:4))).';

	case 'short'
		dcm = [ 1 - 2*q(3)^2 - 2*q(4)^2 	2*(q(2)*q(3) + q(1)*q(4)) 	2*(q(2)*q(4) - q(1)*q(3)) 
				2*(q(2)*q(3) - q(1)*q(4)) 	1 - 2*q(2)^2 - 2*q(4)^2 	2*(q(3)*q(4) + q(1)*q(2))
				2*(q(2)*q(4) + q(1)*q(3)) 	2*(q(3)*q(4) - q(1)*q(2)) 	1 - 2*q(2)^2 - 2*q(3)^2 ];

	otherwise
		error('Not a known form (short or long)');
	end
end