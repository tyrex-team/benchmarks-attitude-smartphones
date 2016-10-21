% This algorithm is a common Extended Kalman Filter but reference and
% 	measured vectors are compared in sensor frame instead of Earth frame.
%
% It has been implemented by T. Michel
%
% This work is a part of project "On Attitude Estimation with Smartphones" 
% http://tyrex.inria.fr/mobile/benchmarks-attitude
%
% Contact :
% Thibaud Michel
% thibaud.michel@gmail.com

classdef QEkfRev < ExtendedKalmanFilter

	methods (Access = public)

		function q = update(obj, gyr, acc, mag, dT)
			q = obj.quaternion.';

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

			dz = [	mag - quatrotate(q_apriori, obj.MagRefNormalized) ...
					acc - quatrotate(q_apriori, obj.AccRefNormalized)].';

			H = [ 	jacobianES(q_apriori, obj.MagRefNormalized)
					jacobianES(q_apriori, obj.AccRefNormalized)];

			R = [obj.noises.magnetometer zeros(3,3) ; zeros(3,3) obj.noises.accelerometer];

		
			K = P_apriori*H.' * (H*P_apriori*H.' + R)^-1;
			q = q_apriori + K * dz;
			P = (eye(4) - K*H) * P_apriori;

			% -- --------- ---


			q = q.'/norm(q);
			obj.quaternion = q;  
			obj.P = P;
		end
	end
end
