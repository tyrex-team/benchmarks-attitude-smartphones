% Copyright Thibaud Michel, Tyrex and NeCS teams, LIG/GIPSA-Lab/INRIA (2016).
% http://tyrex.inria.fr/mobile/benchmarks-attitude
% thibaud.michel@gmail.com

classdef AttitudeBenchmarks < handle

	properties(Access = public)
		
		context = struct; % Context embed lots of information like date, location, magnetic model....

		noises = struct; % Structure of sensors noises.
		calibrations = struct; % Structure of calibration by device and sensors
		datasets = {}; % List of structure: AcquisitionData, MotionData, properties

		toProcess = struct; % list of sampling and calibrations to process

		results; % Results after process

	end

	properties(Access = public, Constant = true)

		coordinateSystem = 'enu';

		url = '../datasets/saved/benchmark.mat';
		
		androidDatasetsPath = '../datasets/android/';
		iosDatasetsPath = '../datasets/ios/';
		qualisysDatasetsPath = '../datasets/qualisys/';

	end

	methods(Static)

		function open()
			disp('Type load(AttitudeBenchmarks.url); in your workspace, the class will be stored in variable benchmark');
		end

		function save()
			directory = fileparts(AttitudeBenchmarks.url); 
			if ~exist(directory, 'dir') mkdir(directory); end
			disp('Type save(AttitudeBenchmarks.url, ''benchmark'', ''-v7.3''); in your workspace');
		end

	end

	methods(Access = public)

		function obj = AttitudeBenchmarks(obj)


			% Location and date properties properties
			location = struct('latitude', 45.187778, 'longitude', 5.726945, 'altitude', 200);
			date = struct('year', 2016, 'month', 05, 'day', 31);


			% Create context from location, date and coordinateSystem
			obj.context = createContextFromLocationAndDate(location, date, obj.coordinateSystem);


			obj.toProcess.samplingRates = [100, 40, 10, 2];


			% osR are data from device calibration if exists or it's from raw data
			% osO are data from device calibration if exists or it's from own calibration data
											% Mag ,  Gyr ,  Acc
			obj.toProcess.calibrations = { 	{'osR', 'osR', 'osR'}, 
											{'own', 'own', 'own'},	
											{'raw', 'raw', 'raw'},	
											{'own', 'raw', 'own'},	
											{'own', 'own', 'raw'},	
											{'own', 'raw', 'raw'},	
											{'own', 'osO', 'own'}};	


			obj.toProcess.algorithms = { ...
						'OS', ...
						'Mahony', 'MahonyB', ...
						'Madgwick', 'MadgwickB', ...
						'Fourati', 'FouratiExtacc', ...
						'Martin', ...
						'Choukroun', 'ChoukrounSn', ...
						'Sabatini', 'SabatiniExtacc', 'SabatiniExtmag', 'SabatiniExtaccExtmag', ...
						'MichelObs', 'MichelObsExtmag','MichelObsExtmagWt', 'MichelObsExtmagWtRep', ...
						'Renaudin', 'RenaudinSn', 'RenaudinB', 'RenaudinBG', 'RenaudinExtacc', 'RenaudinExtmag', 'RenaudinExtaccExtmag', 'RenaudinBGExtaccExtmag', ...
						'Ekf', 'EkfExp', 'EkfLJ', 'EkfSn', 'EkfRev', 'EkfRevSn', ...
						'MichelEkf', 'MichelEkfSn', 'MichelEkfExtmag', 'MichelEkfExtmagWt', 'MichelEkfExtmagWtRep'};

			obj.results = AttitudeBenchmarksResults;

		end

		function load(obj)

			obj.loadNoises;
			obj.loadCalibration;
			obj.loadDatasets;

		end


		function loadNoises(obj)

			disp('------ Load Sensors Noise ------')
			tic;

			% Nexus 5
			obj.noises.n5.accelerometer = [1.60e-02 1.56e-02 2.85e-02].^2;
			obj.noises.n5.gyroscope = [9.38e-04 8.18e-04 7.77e-04].^2;
			obj.noises.n5.magnetometer = [6.50e-01 6.87e-01 6.25e-01].^2;
			obj.noises.n5.gyroscopeBias = [1e-14 7e-08 7e-08 7e-08].^2;

			% iPhone 4S
			obj.noises.i4s.accelerometer = [1.9e-02 1.9e-02 2.3e-02].^2;
			obj.noises.i4s.gyroscope = [2.9e-03 2.7e-03 3.8e-03].^2;
			obj.noises.i4s.magnetometer = [1.2e+00 1.1e+00 1.4e+00].^2;
			obj.noises.i4s.gyroscopeBias = [1e-14 7e-08 7e-08 7e-08].^2;

			% iPhone 5
			obj.noises.i5.accelerometer = [1e-02 1e-02 1e-02].^2;
			obj.noises.i5.gyroscope = [2e-03 2e-03 2e-03].^2;
			obj.noises.i5.magnetometer = [7.5e-01 7.5e-01 8e-01].^2;
			obj.noises.i5.gyroscopeBias = [1e-14 7e-08 7e-08 7e-08].^2;

			toc;
		end


		function loadCalibration(obj)


			disp('------ Load Calibration ------')
			tic

			androPath = obj.androidDatasetsPath;
			iosPath = obj.iosDatasetsPath;

			magMagnitude = obj.context.magnetic.magnitude;
			accMagnitude = obj.context.gravity.magnitude;

			disp('---- Calibration Nexus5 d1 ----');
			s_cm_Nexus5_d1 = loadAndroidDataset([androPath 'calib_mag_Nexus5_d1']);
			s_cg_Nexus5_d1 = loadAndroidDataset([androPath 'calib_gyr_Nexus5_d1']);
			s_ca_Nexus5_d1 = loadAndroidDataset([androPath 'calib_acc_Nexus5_d1']);
			[obj.calibrations.nexus5d1.mag.matrix, obj.calibrations.nexus5d1.mag.bias] = ...
				retrieveParametersFromMagnetometerCalibration(s_cm_Nexus5_d1.rawIMU, magMagnitude);
			[obj.calibrations.nexus5d1.acc.matrix, obj.calibrations.nexus5d1.acc.bias] = ...
				retrieveParametersFromAccelerometerCalibration(s_ca_Nexus5_d1.rawIMU, accMagnitude);
			obj.calibrations.nexus5d1.gyr.bias = retrieveParametersFromGyroscopeCalibration(s_cg_Nexus5_d1.rawIMU);


			disp('---- Calibration Nexus5 d2 ----');
			s_cm_Nexus5_d2 = loadAndroidDataset([androPath 'calib_mag_Nexus5_d2']);
			s_cg_Nexus5_d2 = loadAndroidDataset([androPath 'calib_gyr_Nexus5_d2']);
			s_ca_Nexus5_d2 = loadAndroidDataset([androPath 'calib_acc_Nexus5_d2']);
			[obj.calibrations.nexus5d2.mag.matrix, obj.calibrations.nexus5d2.mag.bias] = ...
				retrieveParametersFromMagnetometerCalibration(s_cm_Nexus5_d2.rawIMU, magMagnitude);
			[obj.calibrations.nexus5d2.acc.matrix, obj.calibrations.nexus5d2.acc.bias] = ...
				retrieveParametersFromAccelerometerCalibration(s_ca_Nexus5_d2.rawIMU, accMagnitude);
			obj.calibrations.nexus5d2.gyr.bias = retrieveParametersFromGyroscopeCalibration(s_cg_Nexus5_d2.rawIMU);

			disp('---- Calibration iPhone4S d2 ----');
			s_cm_iPhone4S_d2 = loadiOSDataset([iosPath 'calib_mag_iPhone4S_d2']);
			s_cg_iPhone4S_d2 = loadiOSDataset([iosPath 'calib_gyr_iPhone4S_d2']);
			s_ca_iPhone4S_d2 = loadiOSDataset([iosPath 'calib_acc_iPhone4S_d2']);
			[obj.calibrations.iPhone4Sd2.mag.matrix, obj.calibrations.iPhone4Sd2.mag.bias] = ...
				retrieveParametersFromMagnetometerCalibration(s_cm_iPhone4S_d2.rawIMU, magMagnitude);
			[obj.calibrations.iPhone4Sd2.acc.matrix, obj.calibrations.iPhone4Sd2.acc.bias] = ...
				retrieveParametersFromAccelerometerCalibration(s_ca_iPhone4S_d2.rawIMU, accMagnitude);
			obj.calibrations.iPhone4Sd2.gyr.bias = retrieveParametersFromGyroscopeCalibration(s_cg_iPhone4S_d2.rawIMU);


			disp('---- Calibration Nexus5 d3 ----');
			s_cm_Nexus5_d3 = loadAndroidDataset([androPath 'calib_mag_Nexus5_d3']);
			s_cg_Nexus5_d3 = loadAndroidDataset([androPath 'calib_gyr_Nexus5_d3']);
			s_ca_Nexus5_d3 = loadAndroidDataset([androPath 'calib_acc_Nexus5_d3']);
			[obj.calibrations.nexus5d3.mag.matrix, obj.calibrations.nexus5d3.mag.bias] = ...
				retrieveParametersFromMagnetometerCalibration(s_cm_Nexus5_d3.rawIMU, magMagnitude);
			[obj.calibrations.nexus5d3.acc.matrix, obj.calibrations.nexus5d3.acc.bias] = ...
				retrieveParametersFromAccelerometerCalibration(s_ca_Nexus5_d3.rawIMU, accMagnitude);
			obj.calibrations.nexus5d3.gyr.bias = retrieveParametersFromGyroscopeCalibration(s_cg_Nexus5_d3.rawIMU);

			disp('---- Calibration iPhone4S d3 ----');
			s_cm_iPhone4S_d3 = loadiOSDataset([iosPath 'calib_mag_iPhone4S_d3']);
			s_cg_iPhone4S_d3 = loadiOSDataset([iosPath 'calib_gyr_iPhone4S_d3']);
			s_ca_iPhone4S_d3 = loadiOSDataset([iosPath 'calib_acc_iPhone4S_d3']);
			[obj.calibrations.iPhone4Sd3.mag.matrix, obj.calibrations.iPhone4Sd3.mag.bias] = ...
				retrieveParametersFromMagnetometerCalibration(s_cm_iPhone4S_d3.rawIMU, magMagnitude);
			[obj.calibrations.iPhone4Sd3.acc.matrix, obj.calibrations.iPhone4Sd3.acc.bias] = ...
				retrieveParametersFromAccelerometerCalibration(s_ca_iPhone4S_d3.rawIMU, accMagnitude);
			obj.calibrations.iPhone4Sd3.gyr.bias = retrieveParametersFromGyroscopeCalibration(s_cg_iPhone4S_d3.rawIMU);

			disp('---- Calibration iPhone5 d3 ----');
			s_cm_iPhone5_d3 = loadiOSDataset([iosPath 'calib_mag_iPhone5_d3']);
			s_cg_iPhone5_d3 = loadiOSDataset([iosPath 'calib_gyr_iPhone5_d3']);
			s_ca_iPhone5_d3 = loadiOSDataset([iosPath 'calib_acc_iPhone5_d3']);
			[obj.calibrations.iPhone5d3.mag.matrix, obj.calibrations.iPhone5d3.mag.bias] = ...
				retrieveParametersFromMagnetometerCalibration(s_cm_iPhone5_d3.rawIMU, magMagnitude);
			[obj.calibrations.iPhone5d3.acc.matrix, obj.calibrations.iPhone5d3.acc.bias] = ...
				retrieveParametersFromAccelerometerCalibration(s_ca_iPhone5_d3.rawIMU, accMagnitude);
			obj.calibrations.iPhone5d3.gyr.bias = retrieveParametersFromGyroscopeCalibration(s_cg_iPhone5_d3.rawIMU);

			disp('---- Calibration Nexus5 d4 ----');
			s_cm_Nexus5_d4 = loadAndroidDataset([androPath 'calib_mag_Nexus5_d4']);
			s_cg_Nexus5_d4 = loadAndroidDataset([androPath 'calib_gyr_Nexus5_d4']);
			s_ca_Nexus5_d4 = loadAndroidDataset([androPath 'calib_acc_Nexus5_d4']);
			[obj.calibrations.nexus5d4.mag.matrix, obj.calibrations.nexus5d4.mag.bias] = ...
				retrieveParametersFromMagnetometerCalibration(s_cm_Nexus5_d4.rawIMU, magMagnitude);
			[obj.calibrations.nexus5d4.acc.matrix, obj.calibrations.nexus5d4.acc.bias] = ...
				retrieveParametersFromAccelerometerCalibration(s_ca_Nexus5_d4.rawIMU, accMagnitude);
			obj.calibrations.nexus5d4.gyr.bias = retrieveParametersFromGyroscopeCalibration(s_cg_Nexus5_d4.rawIMU);

			disp('---- Calibration iPhone4S d4 ----');
			s_cm_iPhone4S_d4 = loadiOSDataset([iosPath 'calib_mag_iPhone4S_d4']);
			s_cg_iPhone4S_d4 = loadiOSDataset([iosPath 'calib_gyr_iPhone4S_d4']);
			s_ca_iPhone4S_d4 = loadiOSDataset([iosPath 'calib_acc_iPhone4S_d4']);
			[obj.calibrations.iPhone4Sd4.mag.matrix, obj.calibrations.iPhone4Sd4.mag.bias] = ...
				retrieveParametersFromMagnetometerCalibration(s_cm_iPhone4S_d4.rawIMU, magMagnitude);
			[obj.calibrations.iPhone4Sd4.acc.matrix, obj.calibrations.iPhone4Sd4.acc.bias] = ...
				retrieveParametersFromAccelerometerCalibration(s_ca_iPhone4S_d4.rawIMU, accMagnitude);
			obj.calibrations.iPhone4Sd4.gyr.bias = retrieveParametersFromGyroscopeCalibration(s_cg_iPhone4S_d4.rawIMU);

			disp('---- Calibration iPhone5 d4 ----');
			s_cm_iPhone5_d4 = loadiOSDataset([iosPath 'calib_mag_iPhone5_d4']);
			s_cg_iPhone5_d4 = loadiOSDataset([iosPath 'calib_gyr_iPhone5_d4']);
			s_ca_iPhone5_d4 = loadiOSDataset([iosPath 'calib_acc_iPhone5_d4']);
			[obj.calibrations.iPhone5d4.mag.matrix, obj.calibrations.iPhone5d4.mag.bias] = ...
				retrieveParametersFromMagnetometerCalibration(s_cm_iPhone5_d4.rawIMU, magMagnitude);
			[obj.calibrations.iPhone5d4.acc.matrix, obj.calibrations.iPhone5d4.acc.bias] = ...
				retrieveParametersFromAccelerometerCalibration(s_ca_iPhone5_d4.rawIMU, accMagnitude);
			obj.calibrations.iPhone5d4.gyr.bias = retrieveParametersFromGyroscopeCalibration(s_cg_iPhone5_d4.rawIMU);

			toc;

		end


		function loadDatasets(obj) 

			disp('------ Load datasets ------');
			tic;

			obj.datasets = {};

			obj.datasets{end+1} = obj.loadTest('Guillaume_iPhone4S_Dist_AR', 'guillaume', 'iPhone4S', true, 'ar', [], 'i4d3');
			obj.datasets{end+1} = obj.loadTest('Guillaume_iPhone4S_Dist_BackPocket', 'guillaume', 'iPhone4S', true, 'backpocket', [], 'i4d3');
			obj.datasets{end+1} = obj.loadTest('Guillaume_iPhone4S_Dist_FrontPocket', 'guillaume', 'iPhone4S', true, 'frontpocket', [], 'i4d3');
			obj.datasets{end+1} = obj.loadTest('Guillaume_iPhone4S_Dist_Phoning', 'guillaume', 'iPhone4S', true, 'phoning', [], 'i4d3');
			obj.datasets{end+1} = obj.loadTest('Guillaume_iPhone4S_Dist_Swinging', 'guillaume', 'iPhone4S', true, 'swinging', [], 'i4d3');
			obj.datasets{end+1} = obj.loadTest('Guillaume_iPhone4S_Dist_Texting', 'guillaume', 'iPhone4S', true, 'texting', [], 'i4d3');
			obj.datasets{end+1} = obj.loadTest('Guillaume_iPhone4S_NoDist_AR', 'guillaume', 'iPhone4S', false, 'ar', [], 'i4d2');
			obj.datasets{end+1} = obj.loadTest('Guillaume_iPhone4S_NoDist_BackPocket', 'guillaume', 'iPhone4S', false, 'backpocket', [21.1 21.5; 23.6 24], 'i4d2');
			obj.datasets{end+1} = obj.loadTest('Guillaume_iPhone4S_NoDist_FrontPocket', 'guillaume', 'iPhone4S', false, 'frontpocket', [], 'i4d2');
			obj.datasets{end+1} = obj.loadTest('Guillaume_iPhone4S_NoDist_Phoning', 'guillaume', 'iPhone4S', false, 'phoning', [], 'i4d2');
			obj.datasets{end+1} = obj.loadTest('Guillaume_iPhone4S_NoDist_RunningHand', 'guillaume', 'iPhone4S', false, 'runninghand', [], 'i4d3');
			obj.datasets{end+1} = obj.loadTest('Guillaume_iPhone4S_NoDist_RunningPocket', 'guillaume', 'iPhone4S', false, 'runningpocket', [], 'i4d3');
			obj.datasets{end+1} = obj.loadTest('Guillaume_iPhone4S_NoDist_Swinging', 'guillaume', 'iPhone4S', false, 'swinging', [], 'i4d2');
			obj.datasets{end+1} = obj.loadTest('Guillaume_iPhone4S_NoDist_Texting', 'guillaume', 'iPhone4S', false, 'texting', [], 'i4d2');

			obj.datasets{end+1} = obj.loadTest('Guillaume_iPhone5_Dist_AR', 'guillaume', 'iPhone5', true, 'ar', [], 'i5d3');
			obj.datasets{end+1} = obj.loadTest('Guillaume_iPhone5_Dist_BackPocket', 'guillaume', 'iPhone5', true, 'backpocket', [], 'i5d3');
			obj.datasets{end+1} = obj.loadTest('Guillaume_iPhone5_Dist_FrontPocket', 'guillaume', 'iPhone5', true, 'frontpocket', [65.6 67.2; 100.4 102], 'i5d3');
			obj.datasets{end+1} = obj.loadTest('Guillaume_iPhone5_Dist_Phoning', 'guillaume', 'iPhone5', true, 'phoning', [], 'i5d3');
			obj.datasets{end+1} = obj.loadTest('Guillaume_iPhone5_Dist_Swinging', 'guillaume', 'iPhone5', true, 'swinging', [103.5 104.5], 'i5d3');
			obj.datasets{end+1} = obj.loadTest('Guillaume_iPhone5_Dist_Texting', 'guillaume', 'iPhone5', true, 'texting', [], 'i5d3');
			obj.datasets{end+1} = obj.loadTest('Guillaume_iPhone5_NoDist_AR', 'guillaume', 'iPhone5', false, 'ar', [], 'i5d3');
			obj.datasets{end+1} = obj.loadTest('Guillaume_iPhone5_NoDist_BackPocket', 'guillaume', 'iPhone5', false, 'backpocket', [5.7 6.1; 7.9 8.5; 9.1 9.6; 24.7 25.3; 26 26.5; 76.4 77.3], 'i5d3');
			obj.datasets{end+1} = obj.loadTest('Guillaume_iPhone5_NoDist_FrontPocket', 'guillaume', 'iPhone5', false, 'frontpocket', [], 'i5d3');
			obj.datasets{end+1} = obj.loadTest('Guillaume_iPhone5_NoDist_Phoning', 'guillaume', 'iPhone5', false, 'phoning', [], 'i5d3');
			obj.datasets{end+1} = obj.loadTest('Guillaume_iPhone5_NoDist_RunningHand', 'guillaume', 'iPhone5', false, 'runninghand', [], 'i5d3');
			obj.datasets{end+1} = obj.loadTest('Guillaume_iPhone5_NoDist_RunningPocket', 'guillaume', 'iPhone5', false, 'runningpocket', [], 'i5d3');
			obj.datasets{end+1} = obj.loadTest('Guillaume_iPhone5_NoDist_Swinging', 'guillaume', 'iPhone5', false, 'swinging', [], 'i5d3');
			obj.datasets{end+1} = obj.loadTest('Guillaume_iPhone5_NoDist_Texting', 'guillaume', 'iPhone5', false, 'texting', [], 'i5d3');

			obj.datasets{end+1} = obj.loadTest('Guillaume_Nexus5_Dist_AR', 'guillaume', 'Nexus5', true, 'ar', [], 'N5d3');
			obj.datasets{end+1} = obj.loadTest('Guillaume_Nexus5_Dist_BackPocket', 'guillaume', 'Nexus5', true, 'backpocket', [], 'N5d3');
			obj.datasets{end+1} = obj.loadTest('Guillaume_Nexus5_Dist_FrontPocket', 'guillaume', 'Nexus5', true, 'frontpocket', [21.2 21.6], 'N5d3');
			obj.datasets{end+1} = obj.loadTest('Guillaume_Nexus5_Dist_Phoning', 'guillaume', 'Nexus5', true, 'phoning', [40.5 40.8], 'N5d3');
			obj.datasets{end+1} = obj.loadTest('Guillaume_Nexus5_Dist_Swinging', 'guillaume', 'Nexus5', true, 'swinging', [59.7 60.5; 73.1 73.5], 'N5d3');
			obj.datasets{end+1} = obj.loadTest('Guillaume_Nexus5_Dist_Texting', 'guillaume', 'Nexus5', true, 'texting', [], 'N5d3');
			obj.datasets{end+1} = obj.loadTest('Guillaume_Nexus5_NoDist_AR', 'guillaume', 'Nexus5', false, 'ar', [64 120], 'N5d2');
			obj.datasets{end+1} = obj.loadTest('Guillaume_Nexus5_NoDist_BackPocket', 'guillaume', 'Nexus5', false, 'backpocket', [], 'N5d2');
			obj.datasets{end+1} = obj.loadTest('Guillaume_Nexus5_NoDist_FrontPocket', 'guillaume', 'Nexus5', false, 'frontpocket', [], 'N5d2');
			obj.datasets{end+1} = obj.loadTest('Guillaume_Nexus5_NoDist_Phoning', 'guillaume', 'Nexus5', false, 'phoning', [], 'N5d2');
			obj.datasets{end+1} = obj.loadTest('Guillaume_Nexus5_NoDist_RunningHand', 'guillaume', 'Nexus5', false, 'runninghand', [], 'N5d4');
			obj.datasets{end+1} = obj.loadTest('Guillaume_Nexus5_NoDist_RunningPocket', 'guillaume', 'Nexus5', false, 'runningpocket', [], 'N5d4');
			obj.datasets{end+1} = obj.loadTest('Guillaume_Nexus5_NoDist_Swinging', 'guillaume', 'Nexus5', false, 'swinging', [], 'N5d2');
			obj.datasets{end+1} = obj.loadTest('Guillaume_Nexus5_NoDist_Texting', 'guillaume', 'Nexus5', false, 'texting', [], 'N5d2');
			


			obj.datasets{end+1} = obj.loadTest('Jakob_iPhone4S_Dist_AR', 'jakob', 'iPhone4S', true, 'ar', [], 'i4d3');
			obj.datasets{end+1} = obj.loadTest('Jakob_iPhone4S_Dist_BackPocket', 'jakob', 'iPhone4S', true, 'backpocket', [], 'i4d3');
			obj.datasets{end+1} = obj.loadTest('Jakob_iPhone4S_Dist_FrontPocket', 'jakob', 'iPhone4S', true, 'frontpocket', [], 'i4d3');
			obj.datasets{end+1} = obj.loadTest('Jakob_iPhone4S_Dist_Phoning', 'jakob', 'iPhone4S', true, 'phoning', [], 'i4d3');
			obj.datasets{end+1} = obj.loadTest('Jakob_iPhone4S_Dist_Swinging', 'jakob', 'iPhone4S', true, 'swinging', [], 'i4d3');
			obj.datasets{end+1} = obj.loadTest('Jakob_iPhone4S_Dist_Texting', 'jakob', 'iPhone4S', true, 'texting', [], 'i4d3');
			obj.datasets{end+1} = obj.loadTest('Jakob_iPhone4S_NoDist_AR', 'jakob', 'iPhone4S', false, 'ar', [], 'i4d2');
			obj.datasets{end+1} = obj.loadTest('Jakob_iPhone4S_NoDist_BackPocket', 'jakob', 'iPhone4S', false, 'backpocket', [], 'i4d2');
			obj.datasets{end+1} = obj.loadTest('Jakob_iPhone4S_NoDist_FrontPocket', 'jakob', 'iPhone4S', false, 'frontpocket', [], 'i4d2');
			obj.datasets{end+1} = obj.loadTest('Jakob_iPhone4S_NoDist_Phoning', 'jakob', 'iPhone4S', false, 'phoning', [], 'i4d2');
			obj.datasets{end+1} = obj.loadTest('Jakob_iPhone4S_NoDist_RunningHand', 'jakob', 'iPhone4S', false, 'runninghand', [34.8 49.2], 'i4d3');
			obj.datasets{end+1} = obj.loadTest('Jakob_iPhone4S_NoDist_RunningPocket', 'jakob', 'iPhone4S', false, 'runningpocket', [], 'i4d3');
			obj.datasets{end+1} = obj.loadTest('Jakob_iPhone4S_NoDist_Swinging', 'jakob', 'iPhone4S', false, 'swinging', [], 'i4d2');
			obj.datasets{end+1} = obj.loadTest('Jakob_iPhone4S_NoDist_Texting', 'jakob', 'iPhone4S', false, 'texting', [], 'i4d2');
			
			obj.datasets{end+1} = obj.loadTest('Jakob_iPhone5_Dist_AR', 'jakob', 'iPhone5', true, 'ar', [], 'i5d3');
			obj.datasets{end+1} = obj.loadTest('Jakob_iPhone5_Dist_BackPocket', 'jakob', 'iPhone5', true, 'backpocket', [], 'i5d3');
			obj.datasets{end+1} = obj.loadTest('Jakob_iPhone5_Dist_FrontPocket', 'jakob', 'iPhone5', true, 'frontpocket', [], 'i5d3');
			obj.datasets{end+1} = obj.loadTest('Jakob_iPhone5_Dist_Phoning', 'jakob', 'iPhone5', true, 'phoning', [], 'i5d3');
			obj.datasets{end+1} = obj.loadTest('Jakob_iPhone5_Dist_Swinging', 'jakob', 'iPhone5', true, 'swinging', [], 'i5d3');
			obj.datasets{end+1} = obj.loadTest('Jakob_iPhone5_Dist_Texting', 'jakob', 'iPhone5', true, 'texting', [], 'i5d3');
			obj.datasets{end+1} = obj.loadTest('Jakob_iPhone5_NoDist_AR', 'jakob', 'iPhone5', false, 'ar', [], 'i5d3');
			obj.datasets{end+1} = obj.loadTest('Jakob_iPhone5_NoDist_BackPocket', 'jakob', 'iPhone5', false, 'backpocket', [], 'i5d3');
			obj.datasets{end+1} = obj.loadTest('Jakob_iPhone5_NoDist_FrontPocket', 'jakob', 'iPhone5', false, 'frontpocket', [], 'i5d3');
			obj.datasets{end+1} = obj.loadTest('Jakob_iPhone5_NoDist_Phoning', 'jakob', 'iPhone5', false, 'phoning', [], 'i5d3');
			obj.datasets{end+1} = obj.loadTest('Jakob_iPhone5_NoDist_RunningHand', 'jakob', 'iPhone5', false, 'runninghand', [16 17; 30 37.5; 39.8 40.2], 'i5d3');
			obj.datasets{end+1} = obj.loadTest('Jakob_iPhone5_NoDist_RunningPocket', 'jakob', 'iPhone5', false, 'runningpocket', [], 'i5d3'); 
			obj.datasets{end+1} = obj.loadTest('Jakob_iPhone5_NoDist_Swinging', 'jakob', 'iPhone5', false, 'swinging', [], 'i5d3');
			obj.datasets{end+1} = obj.loadTest('Jakob_iPhone5_NoDist_Texting', 'jakob', 'iPhone5', false, 'texting', [], 'i5d3');

			obj.datasets{end+1} = obj.loadTest('Jakob_Nexus5_Dist_AR', 'jakob', 'Nexus5', true, 'ar', [], 'N5d2');
			obj.datasets{end+1} = obj.loadTest('Jakob_Nexus5_Dist_BackPocket', 'jakob', 'Nexus5', true, 'backpocket', [], 'N5d2');
			obj.datasets{end+1} = obj.loadTest('Jakob_Nexus5_Dist_FrontPocket', 'jakob', 'Nexus5', true, 'frontpocket', [], 'N5d2');
			obj.datasets{end+1} = obj.loadTest('Jakob_Nexus5_Dist_Phoning', 'jakob', 'Nexus5', true, 'phoning', [4.7 5.6 ; 91.95 93.2], 'N5d2');
			obj.datasets{end+1} = obj.loadTest('Jakob_Nexus5_Dist_Swinging', 'jakob', 'Nexus5', true, 'swinging', [96.2 97.5], 'N5d2');
			obj.datasets{end+1} = obj.loadTest('Jakob_Nexus5_Dist_Texting', 'jakob', 'Nexus5', true, 'texting', [], 'N5d2');
			obj.datasets{end+1} = obj.loadTest('Jakob_Nexus5_NoDist_AR', 'jakob', 'Nexus5', false, 'ar', [], 'N5d1');
			obj.datasets{end+1} = obj.loadTest('Jakob_Nexus5_NoDist_BackPocket', 'jakob', 'Nexus5', false, 'backpocket', [], 'N5d1');
			obj.datasets{end+1} = obj.loadTest('Jakob_Nexus5_NoDist_FrontPocket', 'jakob', 'Nexus5', false, 'frontpocket', [], 'N5d1');
			obj.datasets{end+1} = obj.loadTest('Jakob_Nexus5_NoDist_Phoning', 'jakob', 'Nexus5', false, 'phoning', [], 'N5d1');
			obj.datasets{end+1} = obj.loadTest('Jakob_Nexus5_NoDist_RunningHand', 'jakob', 'Nexus5', false, 'runninghand', [36 47.5], 'N5d3');
			obj.datasets{end+1} = obj.loadTest('Jakob_Nexus5_NoDist_RunningPocket', 'jakob', 'Nexus5', false, 'runningpocket', [], 'N5d3');
			obj.datasets{end+1} = obj.loadTest('Jakob_Nexus5_NoDist_Swinging', 'jakob', 'Nexus5', false, 'swinging', [], 'N5d1');
			obj.datasets{end+1} = obj.loadTest('Jakob_Nexus5_NoDist_Texting', 'jakob', 'Nexus5', false, 'texting', [], 'N5d1');
			


			obj.datasets{end+1} = obj.loadTest('Thibaud_iPhone4S_Dist_AR', 'thibaud', 'iPhone4S', true, 'ar', [], 'i4d3');
			obj.datasets{end+1} = obj.loadTest('Thibaud_iPhone4S_Dist_BackPocket', 'thibaud', 'iPhone4S', true, 'backpocket', [], 'i4d4');
			obj.datasets{end+1} = obj.loadTest('Thibaud_iPhone4S_Dist_FrontPocket', 'thibaud', 'iPhone4S', true, 'frontpocket', [], 'i4d4');
			obj.datasets{end+1} = obj.loadTest('Thibaud_iPhone4S_Dist_Phoning', 'thibaud', 'iPhone4S', true, 'phoning', [], 'i4d3');
			obj.datasets{end+1} = obj.loadTest('Thibaud_iPhone4S_Dist_Swinging', 'thibaud', 'iPhone4S', true, 'swinging', [], 'i4d4');
			obj.datasets{end+1} = obj.loadTest('Thibaud_iPhone4S_Dist_Texting', 'thibaud', 'iPhone4S', true, 'texting', [], 'i4d3');
			obj.datasets{end+1} = obj.loadTest('Thibaud_iPhone4S_NoDist_AR', 'thibaud', 'iPhone4S', false, 'ar', [], 'i4d2');
			obj.datasets{end+1} = obj.loadTest('Thibaud_iPhone4S_NoDist_BackPocket', 'thibaud', 'iPhone4S', false, 'backpocket', [], 'i4d2');
			obj.datasets{end+1} = obj.loadTest('Thibaud_iPhone4S_NoDist_FrontPocket', 'thibaud', 'iPhone4S', false, 'frontpocket', [], 'i4d2');
			obj.datasets{end+1} = obj.loadTest('Thibaud_iPhone4S_NoDist_Phoning', 'thibaud', 'iPhone4S', false, 'phoning', [], 'i4d2');
			obj.datasets{end+1} = obj.loadTest('Thibaud_iPhone4S_NoDist_RunningHand', 'thibaud', 'iPhone4S', false, 'runninghand', [], 'i4d3');
			obj.datasets{end+1} = obj.loadTest('Thibaud_iPhone4S_NoDist_RunningPocket', 'thibaud', 'iPhone4S', false, 'runningpocket', [], 'i4d3');
			obj.datasets{end+1} = obj.loadTest('Thibaud_iPhone4S_NoDist_Swinging', 'thibaud', 'iPhone4S', false, 'swinging', [], 'i4d2');
			obj.datasets{end+1} = obj.loadTest('Thibaud_iPhone4S_NoDist_Texting', 'thibaud', 'iPhone4S', false, 'texting', [51.4 51.8], 'i4d2');

			obj.datasets{end+1} = obj.loadTest('Thibaud_iPhone5_Dist_AR', 'thibaud', 'iPhone5', true, 'ar', [0 2.5; 65.8 66.5; 87.1 87.6], 'i5d3');
			obj.datasets{end+1} = obj.loadTest('Thibaud_iPhone5_Dist_BackPocket', 'thibaud', 'iPhone5', true, 'backpocket', [], 'i5d3');
			obj.datasets{end+1} = obj.loadTest('Thibaud_iPhone5_Dist_FrontPocket', 'thibaud', 'iPhone5', true, 'frontpocket', [52.3 52.8; 61.2 61.7], 'i5d3');
			obj.datasets{end+1} = obj.loadTest('Thibaud_iPhone5_Dist_Phoning', 'thibaud', 'iPhone5', true, 'phoning', [], 'i5d3');
			obj.datasets{end+1} = obj.loadTest('Thibaud_iPhone5_Dist_Swinging', 'thibaud', 'iPhone5', true, 'swinging', [], 'i5d3');
			obj.datasets{end+1} = obj.loadTest('Thibaud_iPhone5_Dist_Texting', 'thibaud', 'iPhone5', true, 'texting', [], 'i5d3');
			obj.datasets{end+1} = obj.loadTest('Thibaud_iPhone5_NoDist_AR', 'thibaud', 'iPhone5', false, 'ar', [], 'i5d3');
			obj.datasets{end+1} = obj.loadTest('Thibaud_iPhone5_NoDist_BackPocket', 'thibaud', 'iPhone5', false, 'backpocket', [], 'i5d3');
			obj.datasets{end+1} = obj.loadTest('Thibaud_iPhone5_NoDist_FrontPocket', 'thibaud', 'iPhone5', false, 'frontpocket', [], 'i5d4');
			obj.datasets{end+1} = obj.loadTest('Thibaud_iPhone5_NoDist_Phoning', 'thibaud', 'iPhone5', false, 'phoning', [], 'i5d3');
			obj.datasets{end+1} = obj.loadTest('Thibaud_iPhone5_NoDist_RunningHand', 'thibaud', 'iPhone5', false, 'runninghand', [], 'i5d3');
			obj.datasets{end+1} = obj.loadTest('Thibaud_iPhone5_NoDist_RunningPocket', 'thibaud', 'iPhone5', false, 'runningpocket', [], 'i5d3');
			obj.datasets{end+1} = obj.loadTest('Thibaud_iPhone5_NoDist_Swinging', 'thibaud', 'iPhone5', false, 'swinging', [], 'i5d3');
			obj.datasets{end+1} = obj.loadTest('Thibaud_iPhone5_NoDist_Texting', 'thibaud', 'iPhone5', false, 'texting', [], 'i5d3');

			obj.datasets{end+1} = obj.loadTest('Thibaud_Nexus5_Dist_AR', 'thibaud', 'Nexus5', true, 'ar', [], 'N5d2');
			obj.datasets{end+1} = obj.loadTest('Thibaud_Nexus5_Dist_BackPocket', 'thibaud', 'Nexus5', true, 'backpocket', [], 'N5d2');
			obj.datasets{end+1} = obj.loadTest('Thibaud_Nexus5_Dist_FrontPocket', 'thibaud', 'Nexus5', true, 'frontpocket', [40 52.6], 'N5d2');
			obj.datasets{end+1} = obj.loadTest('Thibaud_Nexus5_Dist_Phoning', 'thibaud', 'Nexus5', true, 'phoning', [10.8 14; 18 19.6; 44.8 46.4; 87 88; 90.4 91], 'N5d2');
			obj.datasets{end+1} = obj.loadTest('Thibaud_Nexus5_Dist_Swinging', 'thibaud', 'Nexus5', true, 'swinging', [105.45 105.5], 'N5d2');
			obj.datasets{end+1} = obj.loadTest('Thibaud_Nexus5_Dist_Texting', 'thibaud', 'Nexus5', true, 'texting', [], 'N5d2');
			obj.datasets{end+1} = obj.loadTest('Thibaud_Nexus5_NoDist_AR', 'thibaud', 'Nexus5', false, 'ar', [], 'N5d1');
			obj.datasets{end+1} = obj.loadTest('Thibaud_Nexus5_NoDist_BackPocket', 'thibaud', 'Nexus5', false, 'backpocket', [], 'N5d1');
			obj.datasets{end+1} = obj.loadTest('Thibaud_Nexus5_NoDist_FrontPocket', 'thibaud', 'Nexus5', false, 'frontpocket', [], 'N5d1');
			obj.datasets{end+1} = obj.loadTest('Thibaud_Nexus5_NoDist_Phoning', 'thibaud', 'Nexus5', false, 'phoning', [], 'N5d1');
			obj.datasets{end+1} = obj.loadTest('Thibaud_Nexus5_NoDist_RunningHand', 'thibaud', 'Nexus5', false, 'runninghand', [], 'N5d3');
			obj.datasets{end+1} = obj.loadTest('Thibaud_Nexus5_NoDist_RunningPocket', 'thibaud', 'Nexus5', false, 'runningpocket', [], 'N5d3');
			obj.datasets{end+1} = obj.loadTest('Thibaud_Nexus5_NoDist_Swinging', 'thibaud', 'Nexus5', false, 'swinging', [36.8 37.2], 'N5d1');
			obj.datasets{end+1} = obj.loadTest('Thibaud_Nexus5_NoDist_Texting', 'thibaud', 'Nexus5', false, 'texting', [], 'N5d1');
						
			toc;
		end



		function attitudeTest = loadTest(obj, folder, user, deviceName, disturbances, motion, qualisysErrors, calibration)
			% user: thibaud, jakob, guillaume
			% device: iPhone4S, iPhone5, Nexus5
			% disturbances: true, false
			% motion: swinging, phoning, frontpocket, backpocket, texting, ar, runninghand, runningpocket
			% qualisysErrors: [start1 end1; start2 end2], this values have been added manually.

			% Load smartphone measurements
			os = obj.getOsFromDeviceName(deviceName);
			if strcmp(os, 'android')
				datasetSmartphone = loadAndroidDataset([obj.androidDatasetsPath folder]);
			elseif strcmp(os, 'ios')
				datasetSmartphone = loadiOSDataset([obj.iosDatasetsPath folder]);
			end

			% Sensor's attitude from our iPhone app is defined in magnetic north frame 
			qMagneticToTrue = dcm2quat(rotz(obj.context.magnetic.declination));
			datasetSmartphone.attitude(:,2:5) = quatmultiply(qMagneticToTrue, datasetSmartphone.attitude(:,2:5));
			
			% Load reference measurements
			datasetQualisys = loadQualisysDataset([obj.qualisysDatasetsPath folder]);
			datasetQualisys.addBadValues(qualisysErrors);

			% Reference dataset is in enu, let's convert in ned if necessary
			if strcmp(obj.coordinateSystem, 'ned')
				qRotENUToNED = dcm2quat(roty(180)*rotz(90));
				datasetQualisys.attitude = quatmultiply(qRotENUToNED, datasetQualisys.attitude);
			end

			% Dataset alignment
			timeDiff = findTimeDiffBetweenQualisysAndSmartphone(datasetQualisys, datasetSmartphone);
			datasetSmartphone.offsetTimeAllData(timeDiff);
			
			prop = struct('user', user, 'device', deviceName, 'disturbances', disturbances, 'motion', motion, 'calibration', calibration);

			attitudeTest = struct('smartphone', datasetSmartphone, 'reference', datasetQualisys, 'prop', prop);
			
			disp([ folder ' loaded' ]);

		end


		function results = process(obj)


			%%% Pre-process

			numberOfResultsExpected = ((length(obj.toProcess.algorithms) - sum(strcmp(lower(obj.toProcess.algorithms), 'os'))) * ...
										length(obj.toProcess.samplingRates) * length(obj.toProcess.calibrations) + ...
										sum(strcmp(lower(obj.toProcess.algorithms), 'os'))) * length(obj.datasets);
			currentResultProcessed = 0;

			maxSizeOfAlgorithmeName = max(cellfun(@length, obj.toProcess.algorithms));
			maxSizeOfResultsNumber = ceil(log10(numberOfResultsExpected));

			maxSamplingRate = max(obj.toProcess.samplingRates);




			startTic = tic;
			
			for i=1:length(obj.datasets)

				test = obj.datasets{i};
				smartphone = test.smartphone;

				prop = test.prop;


				rawAcc = smartphone.rawIMU.accelerometer;
				rawMag = smartphone.rawIMU.magnetometer;
				rawGyr = smartphone.rawIMU.gyroscope;

				osAcc = smartphone.deviceComputed.accelerometer;
				osMag = smartphone.deviceComputed.magnetometer;
				osGyr = smartphone.deviceComputed.gyroscope;

				calibration = obj.getCalibFromName(prop.calibration);
				ownAcc = obj.calib(rawAcc, calibration.acc.bias, calibration.acc.matrix);
				ownMag = obj.calib(rawMag, calibration.mag.bias, calibration.mag.matrix);
				ownGyr = obj.calib(rawGyr, calibration.gyr.bias);



				for j=1:length(obj.toProcess.calibrations)

					calib = obj.toProcess.calibrations{j};

					switch calib{1}
						case 'raw', mag = rawMag;
						case 'osR', if isempty(osMag), mag = rawMag; else mag = osMag; end
						case 'osO', if isempty(osMag), mag = ownMag; else mag = osMag; end
						case 'own', mag = ownMag;
						otherwise, error('unknown magnetometer calibration');
					end

					switch calib{2}
						case 'raw', gyr = rawGyr;
						case 'osR', if isempty(osGyr), gyr = rawGyr; else gyr = osGyr; end
						case 'osO', if isempty(osGyr), gyr = ownGyr; else gyr = osGyr; end
						case 'own', gyr = ownGyr;
						otherwise, error('unknown gyroscope calibration');
					end

					switch calib{3}
						case 'raw', acc = rawAcc;
						case 'osR', if isempty(osAcc), acc = rawAcc; else acc = osAcc; end
						case 'osO', if isempty(osAcc), acc = ownAcc; else acc = osAcc; end
						case 'own', acc = ownAcc;
						otherwise, error('unknown accelerometer calibration');
					end

							
					firstTimestamp = max([acc(1,1) gyr(1,1) mag(1,1)]);
					lastTimestamp = min([acc(end,1) gyr(end,1) mag(end,1)]);



					for k=1:length(obj.toProcess.samplingRates)

						samplingRate = obj.toProcess.samplingRates(k);
						dT = 1/samplingRate;

						% Align sensors data
						sampling = (firstTimestamp:dT:lastTimestamp)';
						accSampled = interp1(acc(:,1), acc(:,2:4), sampling, 'previous');
						gyrSampled = interp1(gyr(:,1), gyr(:,2:4), sampling, 'previous');
						magSampled = interp1(mag(:,1), mag(:,2:4), sampling, 'previous');


						for l=1:length(obj.toProcess.algorithms)

							algorithm = obj.toProcess.algorithms{l};

							if strcmp(lower(algorithm), 'os') &&  ...
								(~strcmp(calib{1}, 'osR') || ~strcmp(calib{2}, 'osR') || ~strcmp(calib{3}, 'osR') || ...
									samplingRate ~= maxSamplingRate)
								continue;
							end

							result = obj.results.get(prop.user, prop.device, prop.disturbances, ...
								prop.motion, algorithm, calib, samplingRate);
							isResultExists = ~isequal(result, 0);
							
							tic;
							processingTime = 0;

							if ~isResultExists

								if strcmp(lower(algorithm), 'os') ~= 1
									obj.context.noises = obj.getNoisesFromDeviceName(prop.device);
									tic;
									attitude = generateAttitude(sampling, accSampled, gyrSampled, magSampled, ...
										algorithm, obj.context, obj.coordinateSystem);
									processingTime = toc;
								else
									attitude = interpQuaternion(smartphone.attitude(:,1), smartphone.attitude(:,2:5), sampling);
								end
				
								% Get errors from attitude [timestamp qad yaw pitch roll]
								errors = getAttitudeErrors(sampling, attitude, test.reference.timestamp, ...
									test.reference.attitude, test.reference.badValue);

								result = prop;
								result.errors = errors;
								result.algorithm = algorithm;
								result.calibration = calib;
								result.sampling = samplingRate;
								result.processingTime = processingTime;
								result.processingQuaternions = length(sampling);

								obj.results.add(result);
							end

							timeElapsedFromBeginning = toc(startTic);
							currentResultProcessed = currentResultProcessed + 1;
							
							[c firstIndexAfter5Sec] = min(abs(result.errors(:, 1) - 5));

							if ~isResultExists, textSkipOrCompute = 'Comp'; else textSkipOrCompute = 'Skip'; end;
							if prop.disturbances, pertString = 'Yes'; else pertString = 'No'; end
							disp(sprintf(['%s\tuser: %-9s  device: %-8s  Mag. pert.: %-3s' ...
											'  motion: %-13s  calib: %11s (%d)  sampl: %-5s  ' ...
											'algo: %-' num2str(maxSizeOfAlgorithmeName) 's' ...
											 '  Score: %5.1f%c, YPR: %5.1f%c, %4.1f%c, %4.1f%c\t' ...
											 '(%' num2str(maxSizeOfResultsNumber) 'd/%' ...
											 num2str(maxSizeOfResultsNumber) 'd %0.2fs, t:%0.2fs)'], ...
								textSkipOrCompute, prop.user, prop.device, pertString, prop.motion, ...
								sprintf('%s %s %s', calib{:}), j, [num2str(samplingRate) 'Hz'], algorithm, ...
								mean(result.errors(firstIndexAfter5Sec:end,2)), char(176), ...
								mean(abs(result.errors(firstIndexAfter5Sec:end,3))), char(176), ...
								mean(abs(result.errors(firstIndexAfter5Sec:end,4))), char(176), ...
								mean(abs(result.errors(firstIndexAfter5Sec:end,5))), char(176), ...
								currentResultProcessed, numberOfResultsExpected, ...
								processingTime, timeElapsedFromBeginning));
						end
					end
				end
			end

			results = obj.results;
		end




		function calib = getCalibFromName(obj, calibName)

			if strcmp(calibName, 'N5d1')
				calib = obj.calibrations.nexus5d1;
			elseif strcmp(calibName, 'N5d2')
				calib = obj.calibrations.nexus5d2;
			elseif strcmp(calibName, 'i4d2')
				calib = obj.calibrations.iPhone4Sd2;
			elseif strcmp(calibName, 'N5d3')
				calib = obj.calibrations.nexus5d3;
			elseif strcmp(calibName, 'i4d3')
				calib = obj.calibrations.iPhone4Sd3;
			elseif strcmp(calibName, 'i5d3')
				calib = obj.calibrations.iPhone5d3;
			elseif strcmp(calibName, 'N5d4')
				calib = obj.calibrations.nexus5d4;
			elseif strcmp(calibName, 'i4d4')
				calib = obj.calibrations.iPhone4Sd4;
			elseif strcmp(calibName, 'i5d4')
				calib = obj.calibrations.iPhone5d4;
			end
		end

		function noises = getNoisesFromDeviceName(obj, deviceName)
			switch deviceName
				case 'iPhone4S'
					noises = obj.noises.i4s;
				case 'iPhone5'
					noises = obj.noises.i5;
				case 'Nexus5'
					noises = obj.noises.n5;
			end
		end

		function os = getOsFromDeviceName(obj, deviceName)

			if strcmp(deviceName, 'iPhone4S') || strcmp(deviceName, 'iPhone5')
				os = 'ios';
			elseif strcmp(deviceName, 'Nexus5') || strcmp(deviceName, 'Nexus4')
				os = 'android';
			end
		end

		function vec = calib(obj, rawVec, bias, matrix)
		
			vec = rawVec;
			vec(:,2:4) = bsxfun(@minus, vec(:,2:4), bias);

			if exist('matrix')
				for i = 1:size(vec, 1)
					vec(i, 2:4) = (matrix * vec(i, 2:4)')';
				end
			end
		end





		% ----------- Others useful functions -------------

		function showStdOfMagneticFieldMagnitudeOfMeasurements(obj)

			vecWith = [];
			vecWithout = [];

			for i=1:length(obj.datasets)
				d = obj.datasets{i}.smartphone;
				prop = obj.datasets{i}.prop;

				calibration = obj.getCalibFromName(prop.calibration);
				ownMag = obj.calib(d.rawIMU.magnetometer, calibration.mag.bias, calibration.mag.matrix);

				nMag = std(sqrt(sum(ownMag(:,2:4).^2,2)));

				if prop.disturbances
					if nMag < 10
						disp(['Problem with: ' d.datasetLink ', score: ' num2str(nMag)]);
					end
					vecWith(end+1) = nMag;
				else
					if nMag > 10
						disp(['Problem with: ' d.datasetLink ', score: ' num2str(nMag)]);
					end
					vecWithout(end+1) = nMag;
				end
			end

			vecWithout
			vecWith

			disp(['Mean without: ' num2str(mean(vecWithout))]);
			disp(['Mean with: ' num2str(mean(vecWith))]);

		end


		function [statsAccelerations, statsMagneticField] = stats(obj)

			disp(' ')
			disp('External accelerations')
			disp('----------------------')
			statsAccelerations = obj.getExternalAccelerationsStats();
			disp('----------------------')
			disp(' ')
			disp('External magnetic fields')
			disp('------------------------')
			statsMagneticField = obj.getExternalMagneticFieldStats();
			disp('------------------------')
			disp(' ')

		end




		% outputFormat: none, simple (default), html
		function scores = getExternalAccelerationsStats(obj, outputFormat)
			
			if ~exist('outputFormat') outputFormat = 'simple'; end

			scores = [];

			for i=1:length(obj.datasets)
				d = obj.datasets{i}.smartphone;
				prop = obj.datasets{i}.prop;

				if ~isfield(scores, sprintf('%s', prop.motion))
					scores.(prop.motion) = [];
				end

				sampling = obj.datasets{i}.reference.timestamp;
				refQuat = obj.datasets{i}.reference.attitude;
				badValues = obj.datasets{i}.reference.badValue;

				rangeOfGoodValues = zeros(1, length(badValues) - sum(badValues)); indexRange = 1;
				for i = 1:length(badValues)
					if badValues(i) == 0, rangeOfGoodValues(indexRange) = i; indexRange = indexRange + 1; end
				end

				calibration = obj.getCalibFromName(prop.calibration);
				acc = obj.calib(d.rawIMU.accelerometer, calibration.acc.bias, calibration.acc.matrix);

				accSampled = interp1(acc(:,1), acc(:,2:4), sampling, 'linear', 'extrap');

				extAcc = zeros(length(sampling), 3);
				for j=1:length(sampling)
					extAcc(j, :) = quatrotate(quatinv(refQuat(j,:)), accSampled(j,:));
				end
				extAcc = bsxfun(@minus, extAcc(rangeOfGoodValues, :), [0 0 obj.context.gravity.magnitude]); 


				normExtAccRef = abs(sqrt(sum(extAcc.^2,2)));
				scores.(prop.motion)(end+1, 1) = mean(normExtAccRef);


				scores.(prop.motion)(end, 4) = std(normExtAccRef);
				scores.(prop.motion)(end, 5) = sum(normExtAccRef > 0.5)/length(normExtAccRef)*100;
				scores.(prop.motion)(end, 6) = sum(normExtAccRef > 1.5)/length(normExtAccRef)*100;
				scores.(prop.motion)(end, 7) = sum(normExtAccRef > 5)/length(normExtAccRef)*100;


				accSampled = accSampled(rangeOfGoodValues, :);
				normExtAccEstimated = abs(sqrt(sum(accSampled.^2,2)) - obj.context.gravity.magnitude);
				scores.(prop.motion)(end, 2) = mean(normExtAccEstimated);
				scores.(prop.motion)(end, 3) = scores.(prop.motion)(end, 1)/scores.(prop.motion)(end, 2);


				detThreshold = 0.5; refDetection = normExtAccRef > detThreshold; estimatedDetection = normExtAccEstimated > detThreshold;
				scores.(prop.motion)(end, 8) = sum(refDetection & ~estimatedDetection) / length(refDetection) * 100;
				detThreshold = 1.5; refDetection = normExtAccRef > detThreshold; estimatedDetection = normExtAccEstimated > detThreshold;
				scores.(prop.motion)(end, 9) = sum(refDetection & ~estimatedDetection) / length(refDetection) * 100;
				detThreshold = 5; refDetection = normExtAccRef > detThreshold; estimatedDetection = normExtAccEstimated > detThreshold;
				scores.(prop.motion)(end, 10) = sum(refDetection & ~estimatedDetection) / length(refDetection) * 100;
				% sum(~refDetection & estimatedDetection) / length(refDetection); % Should be 0
			end
			

			scores = orderfields(scores, obj.results.displayOrder.motions);

			switch outputFormat

				case 'simple'

					fields = fieldnames(scores);
					disp(sprintf('%13s %10s %12s %10s %10s %10s %10s %10s %10s %10s %10s', ...
						'Motions', 'mae', 'maeEstimated', 'ratio', 'std', '>0.5m.s-2', '>1.5m.s-2', ...
						 '>5m.s-2', 'noDet.>0.5', 'noDet.>1.5', 'noDet.>5'));
					for i = 1:numel(fields)
						vec = mean(scores.(fields{i}));
						disp(sprintf('%13s %10.2f %12.2f %10.2f %10.2f %9.1f%% %9.1f%% %9.1f%% %9.1f%% %9.1f%% %9.1f%%', ...
									fields{i}, vec));
					end

				case 'html'

					fields = fieldnames(scores);
					disp('<tbody>');
					for i = 1:numel(fields)
						disp('	<tr>');
						vec = mean(scores.(fields{i}));
						disp(sprintf(['		<td>%s</td><td>%.2f</td><td>%.2f</td><td>%.2f</td><td>%.2f</td>' ...
										'<td>%.1f%%</td><td>%.1f%%</td><td>%.1f%%</td><td>%.1f%%</td><td>%.1f%%</td><td>%.1f%%</td>'], ...
									fields{i}, vec));
						disp('	</tr>');
					end
					disp('</tbody>');
			end

		end






		% outputFormat: none, simple (default), html
		function scores = getExternalMagneticFieldStats(obj, outputFormat)
			
			if ~exist('outputFormat') outputFormat = 'simple'; end

			scores = [];

			for i=1:length(obj.datasets)
				d = obj.datasets{i}.smartphone;
				prop = obj.datasets{i}.prop;

				if strcmp(prop.motion, 'runninghand') || strcmp(prop.motion, 'runningpocket')
					continue;
				end

				if prop.disturbances, distString = 'with'; else distString = 'without'; end;
				if ~isfield(scores, sprintf('%s', distString))
					scores.(distString) = [];
				end

				sampling = obj.datasets{i}.reference.timestamp;
				refQuat = obj.datasets{i}.reference.attitude;
				badValues = obj.datasets{i}.reference.badValue;

				rangeOfGoodValues = zeros(1, length(badValues) - sum(badValues)); indexRange = 1;
				for i = 1:length(badValues)
					if badValues(i) == 0, rangeOfGoodValues(indexRange) = i; indexRange = indexRange + 1; end
				end

				calibration = obj.getCalibFromName(prop.calibration);
				mag = obj.calib(d.rawIMU.magnetometer, calibration.mag.bias, calibration.mag.matrix);

				magSampled = interp1(mag(:,1), mag(:,2:4), sampling, 'linear', 'extrap');

				extMag = zeros(length(sampling), 3);
				for j=1:length(sampling)
					extMag(j, :) = quatrotate(quatinv(refQuat(j,:)), magSampled(j,:));
				end


				extMag = bsxfun(@minus, extMag(rangeOfGoodValues, :), obj.context.magnetic.vector);


				normExtMagRef = abs(sqrt(sum(extMag.^2,2)));
				scores.(distString)(end+1, 1) = mean(normExtMagRef);

				scores.(distString)(end, 4) = std(normExtMagRef);
				scores.(distString)(end, 5) = sum(normExtMagRef > 10)/length(normExtMagRef)*100;
				scores.(distString)(end, 6) = sum(normExtMagRef > 15)/length(normExtMagRef)*100;
				scores.(distString)(end, 7) = sum(normExtMagRef > 20)/length(normExtMagRef)*100;


				magSampled = magSampled(rangeOfGoodValues, :);
				normExtMagEstimated = abs(sqrt(sum(magSampled.^2,2)) - obj.context.magnetic.magnitude);
				scores.(distString)(end, 2) = mean(normExtMagEstimated);
				scores.(distString)(end, 3) = scores.(distString)(end, 1)/scores.(distString)(end, 2);


				detThreshold = 10; refDetection = normExtMagRef > detThreshold; estimatedDetection = normExtMagEstimated > detThreshold;
				scores.(distString)(end, 8) = sum(refDetection & ~estimatedDetection) / length(refDetection) * 100;
				detThreshold = 15; refDetection = normExtMagRef > detThreshold; estimatedDetection = normExtMagEstimated > detThreshold;
				scores.(distString)(end, 9) = sum(refDetection & ~estimatedDetection) / length(refDetection) * 100;
				detThreshold = 20; refDetection = normExtMagRef > detThreshold; estimatedDetection = normExtMagEstimated > detThreshold;
				scores.(distString)(end, 10) = sum(refDetection & ~estimatedDetection) / length(refDetection) * 100;
				% sum(~refDetection & estimatedDetection) / length(refDetection); % Should be 0
			end
			


			switch outputFormat

				case 'simple'

					fields = fieldnames(scores);
					disp(sprintf('%13s %10s %12s %10s %10s %10s %10s %10s %10s %10s %10s', ...
						'Perturbations', 'mae', 'maeEstimated', 'ratio', 'std', '>10uT', '>15uT', ...
						 '>20uT', 'noDet.>10', 'noDet.>15', 'noDet.>20'));
					for i = 1:numel(fields)
						vec = mean(scores.(fields{i}));
						disp(sprintf('%13s %10.2f %12.2f %10.2f %10.2f %9.1f%% %9.1f%% %9.1f%% %9.1f%% %9.1f%% %9.1f%%', ...
									fields{i}, vec));
					end

				case 'html'

					fields = fieldnames(scores);
					disp('<tbody>');
					for i = 1:numel(fields)
						disp('	<tr>');
						vec = mean(scores.(fields{i}));
						disp(sprintf(['		<td>%s</td><td>%.2f</td><td>%.2f</td><td>%.2f</td><td>%.2f</td>' ...
										'<td>%.1f%%</td><td>%.1f%%</td><td>%.1f%%</td><td>%.1f%%</td><td>%.1f%%</td><td>%.1f%%</td>'], ...
									fields{i}, vec));
						disp('	</tr>');
					end
					disp('</tbody>');
			end
		end

	
	end

end


