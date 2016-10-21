function timeDiff = findTimeDiffBetweenQualisysAndSmartphone(datasetQualisys, datasetSmartphone, force, timeWindow)

	if ~exist('timeWindow')
		timeWindow = 5;
	end

	if ~exist('force')
		force = false;
	end


	s = datasetSmartphone;
	q = datasetQualisys;

	timeAlignmentFile = [s.datasetLink '/timeAlignment.txt'];
	if ~force && exist(timeAlignmentFile, 'file') == 2
		load(timeAlignmentFile,'-ascii');
		timeDiff = timeAlignment;
		return;
	end

	t = q.timestamp;
	pos = sqrt(sum(q.position.^2,2))/1000;
	v = [0 ; diff(pos)./q.dt];
	aQ = [0 ; diff(v)./q.dt];
	aQ(isnan(aQ)) = 0;

	aS = sqrt(sum(s.accelerometer(:,2:4).^2,2));
	aS = aS - mean(aS);


	% plot(s.accelerometer(:,1), aS, t, aQ);

	r = [];

	mRange = -timeWindow:0.01:timeWindow;

	for i=mRange
		
		d = s.accelerometer(:,1) + i;

		aS2 = matchDataSetTimestamp(t, [d aS], 0);
		aS2(:,1) = [];

		r(end+1) = std(aS2-aQ);
	end

	% plot(mRange, r);

	[value, index] = min(r);
	timeDiff = mRange(index);

	save(timeAlignmentFile, 'timeDiff', '-ASCII');
end