function quaternions = positiveQuaternions(quaternions)

	for i = 1:size(quaternions, 1)
		if quaternions(i, 1) < 0
			quaternions(i, :) = -quaternions(i, :);
		end
	end

end