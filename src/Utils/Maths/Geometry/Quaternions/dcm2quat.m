% http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/

function quat = dcm2quat(dcm)

	tr = dcm(1,1) + dcm(2,2) + dcm(3,3);

	if tr > 0

		S = sqrt(tr + 1) * 2;

		quat = [	0.25 * S ...
					(dcm(2,3) - dcm(3,2)) / S ... 
					(dcm(3,1) - dcm(1,3)) / S ...
					(dcm(1,2) - dcm(2,1)) / S];

	elseif (dcm(1,1) > dcm(2,2)) & (dcm(1,1) > dcm(3,3))

		S = sqrt(1 + dcm(1,1) - dcm(2,2) - dcm(3,3)) * 2;

		quat = [	(dcm(2,3) - dcm(3,2)) / S ... 
					0.25 * S ... 
					(dcm(2,1) + dcm(1,2)) / S ...
					(dcm(3,1) + dcm(1,3)) / S];

	elseif dcm(2,2) > dcm(3,3)

		S = sqrt(1 + dcm(2,2) - dcm(1,1) - dcm(3,3)) * 2;

		quat = [	(dcm(3,1) - dcm(1,3)) / S ... 
					(dcm(2,1) + dcm(1,2)) / S ... 
					0.25 * S ...
					(dcm(3,2) + dcm(2,3)) / S];

	else

		S = sqrt(1 + dcm(3,3) - dcm(1,1) - dcm(2,2)) * 2;

		quat = [	(dcm(1,2) - dcm(2,1)) / S ... 
					(dcm(3,1) + dcm(1,3)) / S ... 
					(dcm(3,2) + dcm(2,3)) / S ...
					0.25 * S];

	end

end
