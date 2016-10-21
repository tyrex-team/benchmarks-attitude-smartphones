
% Get attitude errors on 2 non aligned time series of attitude
function errorList = getAttitudeErrors(measuredTimestamps, measured, referenceTimestamps, reference, referenceBadValues)

	%%% Remove measurements before the beginning and after the end of reference: [0 120]

	[~, firstIndexAfter0Sec] = min(abs(measuredTimestamps));
	if measuredTimestamps(firstIndexAfter0Sec) < 0, firstIndexAfter0Sec = firstIndexAfter0Sec + 1; end

	[~, firstIndexAfter120Sec] = min(abs(measuredTimestamps - 120));
	if measuredTimestamps(firstIndexAfter120Sec) > 120, firstIndexAfter120Sec = firstIndexAfter120Sec - 1; end

	measured = measured(firstIndexAfter0Sec:firstIndexAfter120Sec,:);
	measuredTimestamps = measuredTimestamps(firstIndexAfter0Sec:firstIndexAfter120Sec);



	% Align reference and no values on measurements timestamps
	timestamps = measuredTimestamps;
	reference = interpQuaternion(referenceTimestamps, reference, timestamps);
	badValues = interpNoValues(referenceTimestamps, referenceBadValues, timestamps);


	% Get errors in therms of quaternion and yaw pitch roll
	qadError = quaternionAngle(measured, reference) * 180/pi;

	[eulersMeasured(:, 1), eulersMeasured(:, 2), eulersMeasured(:, 3)] = quat2angle(measured);
	[eulersReference(:, 1), eulersReference(:, 2), eulersReference(:, 3)] = quat2angle(reference);
	yprError = abs(diffAngle(eulersMeasured, eulersReference)) * 180/pi;


	% Build range of good values from noValues vector.
	range = zeros(1, length(badValues) - sum(badValues)); indexRange = 1;
	for i = 1:length(badValues)
		if badValues(i) == 0, range(indexRange) = i; indexRange = indexRange + 1; end
	end


	% Build output matrix
	errorList = [ timestamps(range) qadError(range) yprError(range,:) ];

end


function noValuesAligned = interpNoValues(references, noValuesRef, measurements)

	indexMeasurement = 1;
	indexReference = 1;
	sizeMeasurement = length(measurements);
	sizeReference = length(references);


	lastReferenceTimestamp = 0;

	noValuesAligned = zeros(1, sizeMeasurement);
	while indexMeasurement < sizeMeasurement && indexReference < sizeReference
		
		if indexReference + 1 > sizeReference, 
			nextReferenceTimestamp = intmax; 
		else 
			nextReferenceTimestamp = references(indexReference + 1);
		end


		if noValuesRef(indexReference)
			% Remove all measurements from previous reference and until the next one.
			% We do not keep values interpolated by a bad value.

			if indexReference - 1 < 1, 
				lastReferenceTimestamp = 0; 
			else 
				lastReferenceTimestamp = references(indexReference - 1);
			end

			i = 0;
			while indexMeasurement - i > 0 && ...
				measurements(indexMeasurement - i) > lastReferenceTimestamp
				noValuesAligned(indexMeasurement - i) = 1;
				i = i + 1;
			end

			i = 0;
			while indexMeasurement + i <= sizeMeasurement && ...
				measurements(indexMeasurement + i) < nextReferenceTimestamp
				noValuesAligned(indexMeasurement + i) = 1;
				i = i + 1;
			end

		end

		while indexMeasurement <= sizeMeasurement && ...
			measurements(indexMeasurement) < nextReferenceTimestamp
			indexMeasurement = indexMeasurement + 1;
		end

		indexReference = indexReference + 1;
	end

end