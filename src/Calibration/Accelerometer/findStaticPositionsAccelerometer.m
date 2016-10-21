function staticPositions = findStaticPositionsAccelerometer(accelerometer, staticPositionTimeInSecond)

	timestamps = accelerometer(:,1);
	dt = diff(timestamps);
	acceleration = accelerometer(:, 2:4);

    meanDt = mean(dt);
    staticPositionNumberOfIndex = floor(staticPositionTimeInSecond/meanDt);

	movingAverage = centeredMovingAverage(acceleration, meanDt, 0.2);

	% plot(dataset.timestamp, movingAverage, dataset.timestamp, abs(sqrt(sum(dataset.acceleration.^2,2)) - movingAverage));

	staticPositions = [];

	isStatic = false;
	staticCounter = 0;

	for i = 1:length(acceleration)

		diffWithMovingAverage = abs(norm(acceleration(i, :)) - movingAverage(i));

		if diffWithMovingAverage < 0.2

			if ~isStatic
				isStatic = true;
				staticCounter = 1;
			else 
				staticCounter = staticCounter + 1;
			end

		else
			if isStatic
				firstIndex = i - staticCounter;
				lastIndex = i - 1;
				if lastIndex - firstIndex > staticPositionNumberOfIndex
					staticPositions(end+1, :) = [firstIndex lastIndex];
				end
			end

			isStatic = false;
		end

	end

end