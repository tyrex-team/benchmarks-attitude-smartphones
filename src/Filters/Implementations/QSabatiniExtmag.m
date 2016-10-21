classdef QSabatiniExtmag < ExtendedKalmanFilter


	properties(Access = private)

		MagNormThreshold = 15;
		AccMagAngleThreshold = 10;
	end

	% Public methods
	methods (Access = public)



		function q = update(obj, gyr, acc, mag, dT)
			q = obj.quaternion.';

			Rmag = obj.noises.magnetometer;


			% angleAccMag = acos((quatrotate(q, mag) * quatrotate(q, acc))/(norm(acc)*norm(mag)));
			angleAccMag = atan2(norm(cross(acc, mag)), dot(acc, mag));

			if abs(norm(mag) - obj.MagRefNorm) > obj.MagNormThreshold && ...
				abs(angleAccMag - obj.AccMagAngleRef) * 180/pi >  obj.AccMagAngleThreshold
				Rmag = eye(3) * 1e6;
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

			R = [Rmag zeros(3,3) ; zeros(3,3) obj.noises.accelerometer];

		
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