% This algorithm is the same than QEkf
%
% It has been implemented by T. Michel
%
% This work is a part of project "On Attitude Estimation with Smartphones" 
% http://tyrex.inria.fr/mobile/benchmarks-attitude
%
% Contact :
% Thibaud Michel
% thibaud.michel@gmail.com

classdef QMichelEkf < ExtendedKalmanFilter

    methods(Access = public)

		function q = update(obj, gyr, acc, mag, dT)
			% Use the normal filter
			q = obj.updateInternal(gyr, acc, mag, dT);
		end 
    end 


    methods (Access = private)

		function [q, P] = updateInternal(obj, gyr, acc, mag, dT)
			
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

			dz = [ 	obj.MagRefNormalized - quatrotate(q_apriori_inv, mag) ...
					obj.AccRefNormalized - quatrotate(q_apriori_inv, acc)].';

			H = [	jacobianSE(q_apriori, mag)
					jacobianSE(q_apriori, acc)];

			R = [obj.noises.magnetometer zeros(3,3) ; zeros(3,3) obj.noises.accelerometer];

		
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
