classdef ExtendedKalmanFilter < AttitudeFilter 

	properties (Access = protected)

		P = 0.01 * eye(4);

	end


	methods (Access = public)

		function obj = ExtendedKalmanFilter()
			obj = obj@AttitudeFilter();

			obj.noises.accelerometer = 0.5^2 * eye(3);
			obj.noises.magnetometer = 0.8^2 * eye(3);
			obj.noises.gyroscope = 0.3^2 * eye(3);
			obj.noises.gyroscopeBias = 0.0001^2 * eye(3);
		end
	end

	methods(Access = protected, Static = true)

		function output = C(b)
			
			output = [	b(1) -b(2) -b(3) -b(4) ;
						b(2)  b(1)  b(4) -b(3) ;
						b(3) -b(4)  b(1)  b(2) ;
						b(4)  b(3) -b(2)  b(1) ];
		end


		function output = M(b)
			
			output = [	b(1) -b(2) -b(3) -b(4) ;
						b(2)  b(1) -b(4)  b(3) ;
						b(3)  b(4)  b(1) -b(2) ;
						b(4) -b(3)  b(2)  b(1) ];
		end

		function output = h3(q, x)
			q1 = q(1);
			u = q(2:4);
			t = q1*x.' - skew(u) * x.';
			output = 2 * [t dot(u,x) * eye(3) + skew(t)];  
		end
	end

end