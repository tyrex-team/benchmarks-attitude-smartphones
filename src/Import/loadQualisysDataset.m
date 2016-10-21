% Thibaud Michel
% June 2015

function data = loadQualisysDataset(datasetFolder)

	data = MotionData;

	data.datasetLink = datasetFolder;
	w = whos('-file', data.datasetLink);
	m = load(data.datasetLink);	
	d = m.(w(1).name);

	smartphone = d.RigidBodies(1);

	N = d.Frames;

	sampleRate = 1 / d.FrameRate;
	data.timestamp = (0:sampleRate:((N-1)*sampleRate))';
	

	data.attitude = zeros(N, 4);
	data.position = zeros(N, 3);
	data.badValue = zeros(N, 1);

	for i = 1:N
		if sum(isnan(smartphone.Rotations(:, :, i))) == 0
			data.attitude(i,:) = dcm2quat(reshape(smartphone.Rotations(:, :, i), 3, 3)');
		else
			data.badValue(i) = 1;
		end
		data.position(i, :) = smartphone.Positions(:, :, i);
	end

	data.attitude = removeQuaternionsJumps(data.attitude);


end
