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

classdef QMichelEkfExtmagWtRep < ExtendedKalmanFilter

    properties (Access = private)

		MagNormThreshold = 15;

		oldValues = [];
		oldP = [];
		lastMagPerturbation; % in seconds
		magUpdate;

		timeMagNopert = 2; % in seconds
		timeToSavePreviousData = 3; % in seconds


	end
    
    methods(Access = public)
    	
    	function obj = QMichelEkfExtmagWtRep(obj)
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
				
				obj.P = obj.oldP(:,:,1);
				obj.quaternion = obj.oldValues(1, 12:15);

				for i=1:size(obj.oldValues, 1)
					dT = obj.oldValues(i, 2);
					gyr = obj.oldValues(i, 3:5);
					acc = obj.oldValues(i, 6:8);
					mag = obj.oldValues(i, 9:11);
					obj.updateInternal(gyr, acc, mag, dT, false);
				end
			end

			% Use the normal filter
			[q, P] = obj.updateInternal(gyr, acc, mag, dT, magUpdate);


			% Save values in case of reprocessing
			obj.oldValues(end+1,:) = [0 dT gyr acc mag q];
			obj.oldP(:,:,end+1) = P;

			obj.oldValues(:,1) = obj.oldValues(:,1) + dT;
			
			dataToRemove = find(obj.oldValues(:,1) > obj.timeToSavePreviousData)';
			obj.oldValues(dataToRemove, :) = [];
			obj.oldP(:, :, dataToRemove) = [];

			obj.magUpdate = magUpdate;

		end
		
    end 

    methods (Access = private)

		function [q, P] = updateInternal(obj, gyr, acc, mag, dT, magUpdate)
			
			q = obj.quaternion.';

			acc = acc/norm(acc);
			mag = mag/norm(mag);

			if ~magUpdate, Rmag = eye(3) * 1e6; else Rmag = obj.noises.magnetometer; end

			% -- Prediction ---

			F = obj.C([1 0.5 * dT * gyr]);
			q_apriori = F * q; 

			E = [-q(2:4).' ; skew(q(2:4)) + q(1) * eye(3)];
			Qk = (dT / 2)^2 * (E * obj.noises.gyroscope * E.');

			P_apriori = F * obj.P * F.' + Qk;

			% -- --------- ---


			% -- Correction --

			q_apriori_inv = [ q_apriori(1) ; -q_apriori(2:4)];

			dz = [ 	obj.MagRefNormalized - quatrotate(q_apriori_inv, mag) ...
					obj.AccRefNormalized - quatrotate(q_apriori_inv, acc)].';

			H = [	jacobianSE(q_apriori, mag)
					jacobianSE(q_apriori, acc)];

			R = [Rmag zeros(3,3) ; zeros(3,3) obj.noises.accelerometer];

		
			K = P_apriori*H.' * (H*P_apriori*H.' + R)^-1;
			q = q_apriori + K * dz;
			P = (eye(4) - K*H) * P_apriori;

			% -- --------- ---


			q = q.'/norm(q);
			obj.P = P;
			obj.quaternion = q;
		end
	end
end
