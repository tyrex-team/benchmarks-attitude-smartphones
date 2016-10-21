function R = rotz(t)
	
	t = t/180*pi;
    ct = cos(t);
    st = sin(t);
    R = [	ct  -st  0
	        st   ct  0
	        0    0   1];
end