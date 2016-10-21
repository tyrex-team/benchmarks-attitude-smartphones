classdef LinearKalmanFilter < AttitudeFilter

	properties (Access = protected)

		P = 0.01 * eye(4);
		
	end


	methods (Access = public)

		function obj = LinearKalmanFilter()
			obj = obj@AttitudeFilter();

			obj.noises.accelerometer = 0.3^2 * eye(3);
			obj.noises.magnetometer = 0.3^2 * eye(3);
			obj.noises.gyroscope = 0.5^2 * eye(3);

		end

	end

end