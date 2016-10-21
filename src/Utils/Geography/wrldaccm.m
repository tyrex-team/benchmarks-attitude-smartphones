% http://www.npl.co.uk/reference/faqs/how-can-i-determine-my-local-values-of-gravitational-acceleration-and-altitude-(faq-pressure)
function g = wrldaccm(latitude, altitude)

	A = 0.0053024;
	B = 0.0000058;
	L = latitude / 180*pi;
	H = altitude;

	g = 9.780327 * (1 + A * sin(L)^2 - B * sin(2*L)^2) - 3.086 * 10^-6 * H;

end