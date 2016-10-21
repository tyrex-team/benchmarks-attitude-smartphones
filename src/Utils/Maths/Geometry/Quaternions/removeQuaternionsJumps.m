function quaternions = removeQuaternionsJumps(quaternions)

	% This avoid switching quaternion to -quaternion
	for i=2:size(quaternions,1)
		if norm(quaternions(i-1,:) - quaternions(i,:)) > 1
			quaternions(i, :) = - quaternions(i, :);
		end
	end

end