function [matrix, bias] = retrieveParametersFromMagnetometerCalibration(rawIMU, trueMagneticFieldNorm)

	[bias, matrix] = findMagnetometerCalibrationByBartz(rawIMU.magnetometer(:,2:4));

	dataset = AcquisitionData;
	dataset.rawIMU = rawIMU;
	

	% Scale to magnetic field norm
	
	dataset.magnetometer(:,2:4) = bsxfun(@minus, dataset.rawIMU.magnetometer(:,2:4), bias);
	for i=1:size(dataset.magnetometer,1)
		dataset.magnetometer(i, 2:4) = (matrix * dataset.magnetometer(i, 2:4)')';
	end
	measuredMagneticFieldNorm = mean(sqrt(sum(dataset.magnetometer(:,2:4).^2,2)));

	% Grenoble
	% [xyz, h, dec, dip, f] = wrldmagm(200, 45.187778, 5.726945, 2016);
	% trueMagneticFieldNorm = f/1000;

	ratio = trueMagneticFieldNorm/measuredMagneticFieldNorm;
	matrix = matrix * diag([ratio ratio ratio]);


end