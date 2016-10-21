% Thibaud Michel
% December 2015

% Works with gtr4sl and Senslogs
function aData = loadiOSDataset(datasetFolder)

	aData = AcquisitionData;
	aData.provider = 'iOS';

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

	rawIMU.accelerometer(:,2:4) = rawIMU.accelerometer(:,2:4) * Constants.STANDARD_GRAVITY;
	rawIMU.accelerometer(:,2:4) = -rawIMU.accelerometer(:,2:4);

	% Device computed data

	dc = aData.deviceComputed;
	dc.magnetometer = loadIfExists(dl, 'magnetometer-calibrated.txt');
	dc.attitude = loadIfExists(dl, 'attitude.txt');
	dc.magnetometer(:,5) = [];

	headingOfX = 90 * pi / 180;
	headingRotation = [cos(headingOfX) -sin(headingOfX) 	0; ...
			     	   sin(headingOfX)  cos(headingOfX) 	0; ...
			           0           	  	0 					1];
	headingQuaternion = dcm2quat(headingRotation');
	dc.attitude(:,2:5) = quatmultiply(headingQuaternion, dc.attitude(:, 2:5));


	% Copy as default values
	aData.gyroscope = rawIMU.gyroscope;
	aData.accelerometer = rawIMU.accelerometer;
	aData.magnetometer = dc.magnetometer;
	aData.attitude = dc.attitude;

	bootTime = str2num(char(loadProperty(dl, 'Time', 'BootTime')));
	aData.offsetTimeAllData(-bootTime);

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
	filePath = [ datasetLink '/description.txt'];
	if exist(filePath, 'file') == 2
		value = inifile(filePath, 'read', {section, '', key});
	end
end
