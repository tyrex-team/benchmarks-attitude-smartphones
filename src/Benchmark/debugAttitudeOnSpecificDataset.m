% You can une this script to debug your algorithm or visualize attitude, errors and scores

%%% Parameters %%%

name = 'Jakob_iPhone5_NoDist_Texting';
badValues = [];
calibration = 'i5d3';
listOfAlgorithms = {'MadgwickB'};
% listOfAlgorithms = {'OS', 'Mahony', 'Martin'};

showAttitude = false;
showError = false;
showScore = true;

% coordinateSystem can be 'enu' (East-North-Up) or 'ned' (North-East-Down)
coordinateSystem = 'enu';

% Location and date of benchmarks
loc = struct('latitude', 45.187778, 'longitude', 5.726945, 'altitude', 200);
dat = struct('year', 2016, 'month', 05, 'day', 31);

%%% End of parameters %%%



disp(name);

device = lower(calibration(1:2));
if strcmp(device, 'i4'), device = 'i4s'; end

q = loadQualisysDataset(['../datasets/qualisys/' name]);
if device(1) == 'i'
	s = loadiOSDataset(['../datasets/ios/' name]);
else
	s = loadAndroidDataset(['../datasets/android/' name]);
end

coordinateSystem = lower(coordinateSystem);
if ~sum(strmatch(coordinateSystem, {'enu', 'ned'})) error('Unknown coordinate system'); end

switch lower(calibration)
	case 'n5d1'
		s_cm = loadAndroidDataset('../datasets/android/calib_mag_Nexus5_d1');
		s_cg = loadAndroidDataset('../datasets/android/calib_gyr_Nexus5_d1');
		s_ca = loadAndroidDataset('../datasets/android/calib_acc_Nexus5_d1');

	case 'n5d2'
		s_cm = loadAndroidDataset('../datasets/android/calib_mag_Nexus5_d2');
		s_cg = loadAndroidDataset('../datasets/android/calib_gyr_Nexus5_d2');
		s_ca = loadAndroidDataset('../datasets/android/calib_acc_Nexus5_d2');

	case 'n5d3'
		s_cm = loadAndroidDataset('../datasets/android/calib_mag_Nexus5_d3');
		s_cg = loadAndroidDataset('../datasets/android/calib_gyr_Nexus5_d3');
		s_ca = loadAndroidDataset('../datasets/android/calib_acc_Nexus5_d3');

	case 'n5d4'
		s_cm = loadAndroidDataset('../datasets/android/calib_mag_Nexus5_d4');
		s_cg = loadAndroidDataset('../datasets/android/calib_gyr_Nexus5_d4');
		s_ca = loadAndroidDataset('../datasets/android/calib_acc_Nexus5_d4');


	case 'i4d2'
		s_cm = loadiOSDataset('../datasets/ios/calib_mag_iPhone4S_d2');
		s_cg = loadiOSDataset('../datasets/ios/calib_gyr_iPhone4S_d2');
		s_ca = loadiOSDataset('../datasets/ios/calib_acc_iPhone4S_d2');

	case 'i4d3'
		s_cm = loadiOSDataset('../datasets/ios/calib_mag_iPhone4S_d3');
		s_cg = loadiOSDataset('../datasets/ios/calib_gyr_iPhone4S_d3');
		s_ca = loadiOSDataset('../datasets/ios/calib_acc_iPhone4S_d3');

	case 'i4d4'
		s_cm = loadiOSDataset('../datasets/ios/calib_mag_iPhone4S_d4');
		s_cg = loadiOSDataset('../datasets/ios/calib_gyr_iPhone4S_d4');
		s_ca = loadiOSDataset('../datasets/ios/calib_acc_iPhone4S_d4');


	case 'i5d3'
		s_cm = loadiOSDataset('../datasets/ios/calib_mag_iPhone5_d3');
		s_cg = loadiOSDataset('../datasets/ios/calib_gyr_iPhone5_d3');
		s_ca = loadiOSDataset('../datasets/ios/calib_acc_iPhone5_d3');

	case 'i5d4'
		s_cm = loadiOSDataset('../datasets/ios/calib_mag_iPhone5_d4');
		s_cg = loadiOSDataset('../datasets/ios/calib_gyr_iPhone5_d4');
		s_ca = loadiOSDataset('../datasets/ios/calib_acc_iPhone5_d4');

	otherwise
		error('Unknown calibration')
end

% Nexus 5
noises.n5.accelerometer = [1.60e-02 1.56e-02 2.85e-02].^2;
noises.n5.gyroscope = [9.38e-04 8.18e-04 7.77e-04].^2;
noises.n5.magnetometer = [6.50e-01 6.87e-01 6.25e-01].^2;
noises.n5.gyroscopeBias = [7e-08 7e-08 7e-08].^2;

% iPhone4S
noises.i4s.accelerometer = [1.9e-02 1.9e-02 2.3e-02].^2;
noises.i4s.gyroscope = [2.9e-03 2.7e-03 3.8e-03].^2;
noises.i4s.magnetometer = [1.2e+00 1.1e+00 1.4e+00].^2;
noises.i4s.gyroscopeBias = [7e-08 7e-08 7e-08].^2;

% iPhone5
noises.i5.accelerometer = [1e-02 1e-02 1e-02].^2;
noises.i5.gyroscope = [2e-03 2e-03 2e-03].^2;
noises.i5.magnetometer = [7.5e-01 7.5e-01 8e-01].^2;
noises.i5.gyroscopeBias = [7e-08 7e-08 7e-08].^2;


% Dataset alignment
timeDiff = findTimeDiffBetweenQualisysAndSmartphone(q, s);
s.offsetTimeAllData(timeDiff);


% Create context from location, date and coordinateSystem
context = createContextFromLocationAndDate(loc, dat, coordinateSystem);


% Reference dataset is in enu, let's convert in ned if necessary
if strcmp(coordinateSystem, 'ned')
	qRotENUToNED = dcm2quat(roty(180)*rotz(90));
	q.attitude = quatmultiply(qRotENUToNED, q.attitude);
	s.attitude(:,2:5) = quatmultiply(qRotENUToNED, s.attitude(:,2:5));
end

% Sensor's attitude from our iPhone app and from Android app are defined in magnetic north frame 
qMagneticToTrue = dcm2quat(rotz(context.magnetic.declination));
s.attitude(:,2:5) = quatmultiply(qMagneticToTrue, s.attitude(:,2:5));



% Calibration
[matrixMag, biasMag] = retrieveParametersFromMagnetometerCalibration(s_cm.rawIMU, context.magnetic.magnitude);
s.magnetometer = [s.rawIMU.magnetometer(:,1) bsxfun(@minus, s.rawIMU.magnetometer(:,2:4), biasMag)];
for i = 1:size(s.magnetometer, 1), s.magnetometer(i, 2:4) = (matrixMag * s.magnetometer(i, 2:4)')'; end

[matrixAcc, biasAcc] = retrieveParametersFromAccelerometerCalibration(s_ca.rawIMU, context.gravity.magnitude);
s.accelerometer = [s.rawIMU.accelerometer(:,1) bsxfun(@minus, s.rawIMU.accelerometer(:,2:4), biasAcc)];
for i = 1:size(s.accelerometer, 1), s.accelerometer(i, 2:4) = (matrixAcc * s.accelerometer(i, 2:4)')'; end

biasGyr = retrieveParametersFromGyroscopeCalibration(s_cg.rawIMU);
s.gyroscope = [s.rawIMU.gyroscope(:,1) bsxfun(@minus, s.rawIMU.gyroscope(:,2:4), biasGyr)];

q.addBadValues(badValues);




% Align sensors data
acc = s.accelerometer;
gyr = s.gyroscope;
mag = s.magnetometer;

firstTimestamp = max([acc(1,1) gyr(1,1) mag(1,1)]);
lastTimestamp = min([acc(end,1) gyr(end,1) mag(end,1)]);

dT = 1/100;
timestamp = (firstTimestamp:dT:lastTimestamp)';
accSampled = interp1(acc(:,1), acc(:,2:4), timestamp, 'previous');
gyrSampled = interp1(gyr(:,1), gyr(:,2:4), timestamp, 'previous');
magSampled = interp1(mag(:,1), mag(:,2:4), timestamp, 'previous');


% Start the process
attitudeList = {};
errorsList = {};


for k = 1:length(listOfAlgorithms)
	algorithmName = listOfAlgorithms{k};
	if ~strcmp(lower(algorithmName), 'os')
		context.noises = getfield(noises, device);
		tic
		attitudeQ = generateAttitude(timestamp, accSampled, gyrSampled, magSampled, algorithmName, context, coordinateSystem);
		toc
	else
		attitudeQ = interpQuaternion(s.attitude(:,1), s.attitude(:,2:5), timestamp);
	end

	errorQ = getAttitudeErrors(timestamp, attitudeQ, q.timestamp, q.attitude, q.badValue);

	attitudeList{end+1} = attitudeQ;
	errorsList{end+1} = errorQ;
end


if showAttitude
	vectors = {};
	e = zeros(length(q.timestamp), 3);
	[e(:, 1), e(:, 2), e(:, 3)] = quat2angle(q.attitude);
	vectors{end+1} = [q.timestamp e*180/pi];
	% vectors{end+1} = [q.timestamp q.attitude];
	for k = 1:length(errorsList)
		eulers = zeros(size(attitudeList{k}, 1), 3);
	    [eulers(:, 1), eulers(:, 2), eulers(:, 3)] = quat2angle(attitudeList{k});
	    vectors{end+1} = [timestamp eulers*180/pi];
  	    % vectors{end+1} = [timestamp attitudeList{k}];  
	end
	showMatrix('Attitude (eulers)', vectors, ['Reference', listOfAlgorithms], {'yaw' 'pitch' 'roll'}, 'time (s)', 'angle (deg)');
	% showMatrix('Attitude (quaternions)', vectors, ['Reference', listOfAlgorithms], {'w' 'x' 'y' 'z'}, 'time (s)', ' ');
end

if showError
	figure;
	hold on;
	for k = 1:length(errorsList)
		err = errorsList{k};
		plot(err(:,1), err(:,2));
	end
	legend(listOfAlgorithms);
end

if showScore
	for k = 1:length(errorsList)
		err = errorsList{k};
		[c firstIndexAfter5Sec] = min(abs(err(:, 1) - 5));
		disp(sprintf('%s: %0.8f', listOfAlgorithms{k}, mean(err(firstIndexAfter5Sec:end,2))));
	end
end


% % Figure generation about magnitude of magnetic field for paper
% normMag = sqrt(sum(magSampled.^2,2));
% output = [timestamp normMag];
% csvwrite('mag_norm.csv', output);



% % Figure generation about reprocess procedure for paper
% % Jakob_Nexus5_Dist_Phoning
% st = 16;
% en = 38;

% [c firstIndexRef] = min(abs(vectors{1}(:, 1) - st));
% [c lastIndexRef] = min(abs(vectors{1}(:, 1) - en));
% tRef = vectors{1}(firstIndexRef:lastIndexRef, 1);
% sampling = tRef(1):0.03:tRef(end);
% ref = interp1(tRef, vectors{1}(firstIndexRef:lastIndexRef, 2), sampling, 'linear', 'extrap');

% [c firstIndex] = min(abs(vectors{2}(:, 1) - st));
% [c lastIndex] = min(abs(vectors{2}(:, 1) - en));
% t = vectors{2}(firstIndex:lastIndex, 1);
% yaw1 = interp1(t, vectors{2}(firstIndex:lastIndex, 2), sampling, 'linear', 'extrap');
% yaw2 = interp1(t, vectors{3}(firstIndex:lastIndex, 2), sampling, 'linear', 'extrap');
% yaw3 = interp1(t, vectors{4}(firstIndex:lastIndex, 2), sampling, 'linear', 'extrap');

% % outputMat = [sampling.' ref.' yaw1.' yaw2.' yaw3.'];
% % csvwrite('outputReprocess.csv', 'outputMat', '-ascii');
% figure;
% plot(sampling, ref, sampling, yaw1, sampling, yaw2, sampling, yaw3);


