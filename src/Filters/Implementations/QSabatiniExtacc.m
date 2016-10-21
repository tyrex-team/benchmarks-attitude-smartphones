classdef QSabatiniExtacc < ExtendedKalmanFilter


	properties(Access = private)

		AccNormThreshold = 0.5;
		movingAverageWindowNormAcc = 0.1;

		movingAverageNormAcc;
	end


	methods (Access = public)

		function notifyReferenceVectorChanged(obj)
			notifyReferenceVectorChanged@AttitudeFilter(obj);

    		obj.movingAverageNormAcc = [0 obj.AccRefNorm];
		end

		function q = update(obj, gyr, acc, mag, dT)
			q = obj.quaternion.';

			Racc = obj.noises.accelerometer;

			obj.movingAverageNormAcc(:,1) = obj.movingAverageNormAcc(:,1) + dT;
			obj.movingAverageNormAcc(find(obj.movingAverageNormAcc(:,1) > obj.movingAverageWindowNormAcc),:) = [];
			obj.movingAverageNormAcc(end+1,:) = [0 norm(acc)];
			
			eAcc = mean(abs(obj.movingAverageNormAcc(:,2) - obj.AccRefNorm));
			if eAcc > obj.AccNormThreshold
				Racc = eye(3) * 1e6;
			end

			acc = acc/norm(acc);
			mag = mag/norm(mag);

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
					obj.AccRefNormalized - quatrotate(q_apriori_inv, acc)]';

			H = [	jacobianSE(q_apriori, mag)
					jacobianSE(q_apriori, acc)];

			R = [obj.noises.magnetometer zeros(3,3) ; zeros(3,3) Racc];

		
			K = P_apriori*H.' * (H*P_apriori*H.' + R)^-1;
			q = q_apriori + K * dz;
			P = (eye(4) - K*H) * P_apriori;

			% -- --------- ---


			q = q/norm(q);
			q = q.';
			obj.quaternion = q;  
			obj.P = P;
		end
	end

end