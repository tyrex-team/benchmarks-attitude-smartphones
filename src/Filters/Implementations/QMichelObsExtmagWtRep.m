% This algorithm is based on QMichelEkfExtMagWt and the reprocess phase 
%	has been added
%
% This algorithm has been implemented by T. Michel
%
% This work is a part of project "On Attitude Estimation with Smartphones" 
% http://tyrex.inria.fr/mobile/benchmarks-attitude
%
% Contact :
% Thibaud Michel
% thibaud.michel@gmail.com

classdef QMichelObsExtmagWtRep < AttitudeFilter

    properties (Access = private)

		MagNormThreshold = 15;

		Beta = 0.3;
		Ka = 2;
		Km = 1;


		oldValues = [];
		lastMagPerturbation; % in seconds
		magUpdate;

		timeMagNopert = 2; % in seconds
		timeToSavePreviousData = 3; % in seconds


	end
    
    methods(Access = public)
    	
    	function obj = QMichelObsExtmagWtRep(obj)
    		obj.lastMagPerturbation = obj.timeMagNopert;
    		obj.magUpdate = false;
    	end


		function q = update(obj, gyr, acc, mag, dT)

			magUpdate = abs( norm(mag) - obj.MagRefNorm) < obj.MagNormThreshold;

			% Do not consider a magUpdate for next [timeMagNopert] seconds if a ~magUpdate is detected
			obj.lastMagPerturbation(~magUpdate) = 0;
			obj.lastMagPerturbation(magUpdate) = obj.lastMagPerturbation + dT;
			magUpdate(obj.lastMagPerturbation < obj.timeMagNopert) = false;


			% If we detect a perturbation, we replay all previous values from current time - [timeToSavePreviousData]
			% to current time without magnetic field updates
			if obj.magUpdate && ~magUpdate

				tmpBeta = obj.Beta;
				obj.Beta = 2;
				obj.quaternion = obj.oldValues(1, 12:15);

				for i=1:size(obj.oldValues, 1)
					dT = obj.oldValues(i, 2);
					gyrOld = obj.oldValues(i, 3:5);
					accOld = obj.oldValues(i, 6:8);
					magOld = obj.oldValues(i, 9:11);
					obj.updateInternal(gyrOld, accOld, magOld, dT, false);
				end
				obj.Beta = tmpBeta;
			end


			% Save values in case of reprocessing
			obj.oldValues(end+1,:) = [0 dT gyr acc mag obj.quaternion];
			obj.oldValues(:,1) = obj.oldValues(:,1) + dT;
			obj.oldValues(find(obj.oldValues(:,1) > obj.timeToSavePreviousData),:) = [];


			% Use the normal filter
			q = obj.updateInternal(gyr, acc, mag, dT, magUpdate);

			obj.magUpdate = magUpdate;
		end
		
    end 

    methods (Access = private)

		function q = updateInternal(obj, gyr, acc, mag, dT, magUpdate)
			
			q = obj.quaternion;

			acc = acc/norm(acc);
			mag = mag/norm(mag);

			estimate_A = quatrotate(q, obj.AccRefNormalized);
			estimate_M = quatrotate(q, obj.MagRefNormalized);
			
			Measure = [acc  mag];
			Estimate = [estimate_A  estimate_M];

			delta = 2 * [obj.Ka * skew(estimate_A) ; obj.Km * magUpdate * skew(estimate_M)]';

			% Gradient decent algorithm corrective step
			dq = (Measure - Estimate) * ((delta * delta' + 1e-5 * eye(3))^-1 * delta)';

			qDot = 0.5 * quatmultiply(q, [0 gyr]) + obj.Beta * quatmultiply(q, [0 dq]);
			q = q + qDot * dT;
			q = q/norm(q);

			obj.quaternion = q;
		end

	end
end
