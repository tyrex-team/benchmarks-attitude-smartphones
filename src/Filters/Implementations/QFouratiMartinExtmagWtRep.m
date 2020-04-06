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

classdef QFouratiMartinExtmagWtRep < AttitudeFilter

    properties (Access = private)

		MagNormThreshold = 15;

		Beta = 0.3;
		Ka = 2;
		Kc = 1;

		oldValues = [];
		lastMagPerturbation; % in seconds
		magUpdate;

		timeMagNopert = 2; % in seconds
		timeToSavePreviousData = 3; % in seconds

	end

	properties (Access = private)
		CRef;
		CRefNormalized;
	end
    
    methods(Access = public)
    	
    	function obj = QFouratiMartinExtmagWtRep(obj)
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
				obj.Beta = 0.05;
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
		
		function setParams(obj, params)
			obj.Beta = params(1);
			obj.Ka = params(2);
			obj.Kc = params(3);
			obj.MagNormThreshold = params(4);
			obj.timeMagNopert = params(5);
			obj.timeToSavePreviousData = params(6);
		end

		function notifyReferenceVectorChanged(obj)

			notifyReferenceVectorChanged@AttitudeFilter(obj)

			obj.CRef = cross(obj.AccRef, obj.MagRef);
    		obj.CRefNormalized = obj.CRef/norm(obj.CRef);

		end

    end 

    methods (Access = private)
    
		function q = updateInternal(obj, gyr, acc, mag, dT, magUpdate)
			
			q = obj.quaternion;	

			acc = acc / norm(acc);
 			mag = mag / norm(mag); 
			c = cross(acc, mag);
			c = c/norm(c);

            estimate_A = quatrotate(q, obj.AccRefNormalized);
            estimate_C = quatrotate(q, obj.CRefNormalized);


            if magUpdate
				Measure = [acc  c];
				Estimate = [estimate_A  estimate_C];
				delta = 2 * [obj.Ka * skew(estimate_A) ; obj.Kc * skew(estimate_C)]';
			else
				Measure = acc;
				Estimate = estimate_A;	
				delta = 2 * [obj.Ka * skew(estimate_A)]';
			end

			

			% Gradient decent algorithm corrective step
			dq = (Measure - Estimate) * ((delta * delta' + 1e-5 * eye(3))^-1 * delta)';

			qDot = 0.5 * quatmultiply(q, [0 gyr]) + obj.Beta * quatmultiply(q, [0 dq]);
			q = q + qDot * dT;
			q = q/norm(q);

			obj.quaternion = q;
		end

	end
end
