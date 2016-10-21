function staticPositions = findStaticPositionsGyroscope(gyroscope, staticPositionTimeInSecond)

	timestamps = gyroscope(:,1);
	dt = diff(timestamps);
	rotationRate = gyroscope(:, 2:4);

    meanDt = mean(dt);
	movingAverage = centeredMovingAverage(rotationRate, meanDt, 0.2);

	% plot( ...
	% 	timestamps, abs(sqrt(sum(rotationRate.^2,2))), 'b', ...
	% 	timestamps, abs(sqrt(sum(rotationRate.^2,2)) - movingAverage), 'g', ...
	% 	timestamps, movingAverage, 'r');

	staticPositions = [];

	isStatic = false;
	staticCounter = 0;

	len = length(timestamps);

	for i = 1:len

		if movingAverage(i,:) < 0.15

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
				staticPositions(end+1, :) = [firstIndex lastIndex];
			end

			isStatic = false;
		end

	end

	if isStatic
		staticPositions(end+1, :) = [len-staticCounter+1 len];
	end

	removeGapTime = 0.3;

	i = 2;
	while i <= size(staticPositions,1)
		if size(staticPositions,1) > 1 && timestamps(staticPositions(i,1)) - timestamps(staticPositions(i-1,2)) < removeGapTime
			staticPositions(i-1,2) = staticPositions(i,2);
			staticPositions(i,:) = [];
		elseif timestamps(staticPositions(i-1,2)) - timestamps(staticPositions(i-1,1)) < staticPositionTimeInSecond
			staticPositions(i-1,:) = [];			
		else
			i = i + 1;
		end
	end


end