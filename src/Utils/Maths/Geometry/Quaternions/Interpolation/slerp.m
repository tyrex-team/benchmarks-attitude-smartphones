% http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/slerp/

function qm = slerp(qa, qb, t)

	% Calculate angle between them.
	cosHalfTheta = dot(qa, qb);
	
	% if qa = qb or qa = -qb then theta = 0 and we can return qa
	if abs(cosHalfTheta) >= 1.0-1e-10
		qm = qa;
	else
		% Calculate temporary values.
		halfTheta = acos(cosHalfTheta);

		sinHalfTheta = sqrt(1.0 - cosHalfTheta.^2);

		% if theta = 180 degrees then result is not fully defined
		% we could rotate around any axis normal to qa or qb
		if abs(sinHalfTheta) < 0.001
			qm = 0.5 * (qa + qb);
		else
			ratioA = sin((1 - t) * halfTheta) / sinHalfTheta;
			ratioB = sin(t * halfTheta) / sinHalfTheta; 
			
			%calculate Quaternion.
			qm = qa * ratioA + qb * ratioB;
			
		end
	end
end

