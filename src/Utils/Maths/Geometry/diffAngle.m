% diff, angle1 and angle2 in radians
function diff = diffAngle(angle1, angle2)
	diff = mod(angle1 - angle2 + pi, 2*pi) - pi;
end