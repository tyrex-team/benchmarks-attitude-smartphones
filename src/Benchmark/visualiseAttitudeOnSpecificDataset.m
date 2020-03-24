% You can une this script to visualise attitude (without calibration) of a specific dataset

%%% Parameters %%%

name = 'Thibaud_Nexus5_Dist_Texting';

listOfAlgorithms = {'OS', 'Mahony', 'Madgwick', 'Ekf', 'EkfWithoutMag'};

% coordinateSystem can be 'enu' (East-North-Up) or 'ned' (North-East-Down)
coordinateSystem = 'enu';

% Location and date of benchmarks
loc = struct('latitude', 45.187778, 'longitude', 5.726945, 'altitude', 200);
dat = struct('year', 2016, 'month', 05, 'day', 31);

%%% End of parameters %%%

disp(name);

% s = loadiOSDataset(['../datasets/ios/' name]);
s = loadAndroidDataset(['../datasets/android/' name]);


coordinateSystem = lower(coordinateSystem);
if ~sum(strmatch(coordinateSystem, {'enu', 'ned'})) error('Unknown coordinate system'); end


% Create context from location, date and coordinateSystem
context = createContextFromLocationAndDate(loc, dat, coordinateSystem);


% Reference dataset is in enu, let's convert in ned if necessary
if strcmp(coordinateSystem, 'ned')
	qRotENUToNED = dcm2quat(roty(180)*rotz(90));
	s.attitude(:,2:5) = quatmultiply(qRotENUToNED, s.attitude(:,2:5));
end

% Sensor's attitude from our iPhone app and from Android app are defined in magnetic north frame 
qMagneticToTrue = dcm2quat(rotz(context.magnetic.declination));
s.attitude(:,2:5) = quatmultiply(qMagneticToTrue, s.attitude(:,2:5));



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
vecQuaternions = {};
vecEulerAngles = {};

for k = 1:length(listOfAlgorithms)
	algorithmName = listOfAlgorithms{k};
	if ~strcmpi(algorithmName, 'os')
		attitudeQ = generateAttitude(timestamp, accSampled, gyrSampled, magSampled, algorithmName, context, coordinateSystem);
	else
		attitudeQ = interpQuaternion(s.attitude(:,1), s.attitude(:,2:5), timestamp);
	end

	vecQuaternions{end+1} = [timestamp positiveQuaternions(attitudeQ)];

	eulerAngles = zeros(size(attitudeQ, 1), 3);
	[eulerAngles(:, 1), eulerAngles(:, 2), eulerAngles(:, 3)] = quat2angle(attitudeQ);
	vecEulerAngles{end+1} = [timestamp eulerAngles*180/pi];
end

showMatrix('Attitude (quaternions)', vecQuaternions, listOfAlgorithms, {'w' 'x' 'y' 'z'}, 'time (s)', '', [-1, 1]);
showMatrix('Attitude (eulers)', vecEulerAngles, listOfAlgorithms, {'yaw' 'pitch' 'roll'}, 'time (s)', 'angle (deg)', [-180, 180]);
