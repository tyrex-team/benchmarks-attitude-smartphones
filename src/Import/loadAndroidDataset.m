
function aData = loadAndroidDataset(datasetFolder)

	aData = AcquisitionData;
	aData.provider = 'Android';

	aData.datasetLink = datasetFolder;
	dl = aData.datasetLink; % shorter var name

	if exist(dl, 'file') ~= 7 
		error([dl ' does not exists'])
	end


	% Raw data
	rawIMU = aData.rawIMU;
	rawIMU.gyroscope = loadIfExists(dl, 'gyroscope.txt');
	rawIMU.accelerometer = loadIfExists(dl, 'accelerometer.txt');
	rawIMU.magnetometer = loadIfExists(dl, 'magnetometer.txt');
	rawIMU.gyroscope(:, 5:end) = [];
	rawIMU.magnetometer(:, 5:end) = [];



	% Device computed data
	dc = aData.deviceComputed;
	dc.gyroscope = loadIfExists(dl, 'gyroscope-calibrated.txt');
	dc.magnetometer = loadIfExists(dl, 'magnetometer-calibrated.txt');
	dc.attitude = loadIfExists(dl, 'rotation-vector.txt');
	
	% Quaternion gave by Android is [q2 q3 q4 q1]
	% http://developer.android.com/reference/android/hardware/SensorEvent.html#values
	if ~isempty(dc.attitude)
		dc.attitude(:,6) = []; 
		dc.attitude(:, 2:5) = dc.attitude(:, [5 2 3 4]);
		dc.attitude(:, 2:5) = removeQuaternionsJumps(dc.attitude(:, 2:5));
	end


	% Copy as default values
	aData.gyroscope = dc.gyroscope;
	aData.accelerometer = rawIMU.accelerometer;
	aData.magnetometer = dc.magnetometer;
	aData.attitude = dc.attitude;
	
	aData.unixTime = datenum('01-jan-1970 00:00:00') + str2num(char(loadProperty(dl, 'Time', 'StartTime')))/86400;
end

function outputMatrix = loadIfExists(datasetLink, fileName, removeFirstColumn)

	if ~exist('removeFirstColumn')
		removeFirstColumn = true;
	end

	filePath = [ datasetLink '/' fileName];
	if exist(filePath, 'file') == 2
		[A,delimiterOut,headerlinesOut] = importdata(filePath,' ');
		if isstruct(A)
			outputMatrix = A.data;
			if removeFirstColumn && size(outputMatrix,2) > 1
				outputMatrix(:,1) = [];
			end
		elseif ismatrix(A)
			outputMatrix = A;
		else
			outputMatrix = [];
		end
	else
		outputMatrix = [];
	end

end

function value = loadProperty(datasetLink, section, key)
	filePath = [ datasetLink '/record.properties'];
	if exist(filePath, 'file') == 2
		value = inifile(filePath, 'read', {section, '', key});
	end
end


