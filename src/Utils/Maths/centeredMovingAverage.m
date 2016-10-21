
function outputVector = centeredMovingAverage(m, dt, movAverageWindowSizeSeconds)

	movAverageWindowSize = floor(movAverageWindowSizeSeconds / dt / 2);

	if size(m, 1) > 1
		v = sqrt(sum(m.^2,2));
	else
		v = m;
	end

	N = length(v);
	outputVector = zeros(N, 1);
	for i = 1:N
		startIndex = max(1, i - movAverageWindowSize);
		endIndex = min(N, i + movAverageWindowSize);
		outputVector(i) = mean(v(startIndex:endIndex));
	end


end