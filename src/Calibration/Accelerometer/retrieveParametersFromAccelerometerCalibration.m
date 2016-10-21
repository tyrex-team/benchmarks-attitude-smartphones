function [matrix, bias] = retrieveParametersFromAccelerometerCalibration(rawIMU, accelerationMagnitude)

	accelerometer = rawIMU.accelerometer;
	
	% Retrieve static values
	staticRanges = findStaticPositionsAccelerometer(accelerometer, 1); % static positions for 1 sec
	staticValues = [];
	for i = 1 : length(staticRanges)
		values = accelerometer(staticRanges(i, 1):staticRanges(i, 2), 2:4);
		staticValues(end+1, :) = mean(values);
	end

	[matrix, bias] = findAccelerometerCalibrationByKumar(staticValues);

	matrix = matrix .* accelerationMagnitude;
end