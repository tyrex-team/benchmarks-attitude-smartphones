
classdef AcquisitionData < handle

	properties(Access = public)

		provider = 'Unknown'
		datasetLink
	    unixTime

		%  === Raw ===
		rawIMU

		%  === Device computed ===
		deviceComputed

		% == Used by code (default is from device computed) ==
		magnetometer 
		gyroscope
		accelerometer

		attitude
	end
	

	methods(Access = public)

		function obj = AcquisitionData(obj)
			obj.rawIMU = IMUData;
			obj.deviceComputed = DeviceComputedData;
		end


		function offsetTimeAllData(obj, timeDiff)

			if ~isempty(obj.accelerometer) obj.accelerometer(:,1) = obj.accelerometer(:,1) + timeDiff; end
			if ~isempty(obj.magnetometer) obj.magnetometer(:,1) = obj.magnetometer(:,1) + timeDiff; end
			if ~isempty(obj.gyroscope) obj.gyroscope(:,1) = obj.gyroscope(:,1) + timeDiff; end
			if ~isempty(obj.attitude) obj.attitude(:,1) = obj.attitude(:,1) + timeDiff; end
			if ~isempty(obj.rawIMU.accelerometer) obj.rawIMU.accelerometer(:,1) = obj.rawIMU.accelerometer(:,1) + timeDiff; end
			if ~isempty(obj.rawIMU.magnetometer) obj.rawIMU.magnetometer(:,1) = obj.rawIMU.magnetometer(:,1) + timeDiff; end
			if ~isempty(obj.rawIMU.gyroscope) obj.rawIMU.gyroscope(:,1) = obj.rawIMU.gyroscope(:,1) + timeDiff; end
			if ~isempty(obj.deviceComputed.accelerometer) obj.deviceComputed.accelerometer(:,1) = obj.deviceComputed.accelerometer(:,1) + timeDiff; end
			if ~isempty(obj.deviceComputed.magnetometer) obj.deviceComputed.magnetometer(:,1) = obj.deviceComputed.magnetometer(:,1) + timeDiff; end
			if ~isempty(obj.deviceComputed.gyroscope) obj.deviceComputed.gyroscope(:,1) = obj.deviceComputed.gyroscope(:,1) + timeDiff; end
			if ~isempty(obj.deviceComputed.attitude) obj.deviceComputed.attitude(:,1) = obj.deviceComputed.attitude(:,1) + timeDiff; end

		end

	end
end
