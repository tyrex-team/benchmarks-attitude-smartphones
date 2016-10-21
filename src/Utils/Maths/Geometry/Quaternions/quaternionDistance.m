function distance = quaternionDistance(q1, q2)
	distance = 1 - (q1*q2')^2;
end