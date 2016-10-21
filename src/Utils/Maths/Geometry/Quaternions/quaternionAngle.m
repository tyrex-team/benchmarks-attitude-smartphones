function r = quaternionAngle(q1, q2)
	% r = acos(2 * (q1*q2')^2 -1);
	r = acos(2 * (q1(:,1).*q2(:,1)+q1(:,2).*q2(:,2)+q1(:,3).*q2(:,3)+q1(:,4).*q2(:,4)).^2 -1);
end