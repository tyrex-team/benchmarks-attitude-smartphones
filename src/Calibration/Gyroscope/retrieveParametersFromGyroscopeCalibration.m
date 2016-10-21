function bias = retrieveParametersFromGyroscopeCalibration(rawIMU)

	gyroscope = rawIMU.gyroscope;

	staticRanges = findStaticPositionsGyroscope(gyroscope, 2); % static positions for 2 sec
	staticValues = [];

	for i = 1 : size(staticRanges,1)
		staticValues = [ staticValues ; gyroscope(staticRanges(i, 1):staticRanges(i, 2), 2:4)];
	end

	bias = mean(staticValues);

end