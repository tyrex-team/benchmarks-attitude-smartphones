% This algorithm is a common Extended Kalman Filter using long form of jacobian 
% 	instead of short form. See Filters/Implementations/Utils/understandJacobians.m 
% 	for more information.
%
% It has been implemented by T. Michel
%
% This work is a part of project "On Attitude Estimation with Smartphones" 
% http://tyrex.inria.fr/mobile/benchmarks-attitude
%
% Contact :
% Thibaud Michel
% thibaud.michel@gmail.com

classdef QEkfLongJacobian < ExtendedKalmanFilter

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

			q_apriori_inv = [ q_apriori(1) ; -q_apriori(2:4)];

			dz = [ 	obj.MagRefNormalized - quatrotate(q_apriori_inv, mag, 'long') ...
					obj.AccRefNormalized - quatrotate(q_apriori_inv, acc, 'long')].';

			H = [	jacobianSE(q_apriori, mag, 'long')
					jacobianSE(q_apriori, acc, 'long')];

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
