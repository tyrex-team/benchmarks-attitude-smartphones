% This algorithm is a common Extended Kalman Filter without magnetometer
%
% It has been implemented by T. Michel
%
% This work is a part of project "On Attitude Estimation with Smartphones" 
% http://tyrex.inria.fr/mobile/benchmarks-attitude
%
% Contact :
% Thibaud Michel
% thibaud.michel@gmail.com

classdef QEkfWithoutMag < ExtendedKalmanFilter

	methods (Access = public)

		function q = update(obj, gyr, acc, mag, dT)
			q = obj.updateInternal(gyr, acc, dT);
		end

		function q = updateInternal(obj, gyr, acc, dT)
			q = obj.quaternion.';
				
			acc = acc/norm(acc);
			% -- Prediction ---

			F = obj.C([1 0.5 * dT * gyr]);
			q_apriori = F * q;

			E = [-q(2:4).' ; skew(q(2:4)) + q(1) * eye(3)];
			Qk = (dT / 2)^2 * (E * obj.noises.gyroscope * E.');

			P_apriori = F * obj.P * F.' + Qk;
			% -- --------- ---



			% -- Correction --

			dz = [acc - quatrotate(q_apriori, obj.AccRefNormalized)].';

			H = [jacobianES(q_apriori, obj.AccRefNormalized)];

			R = [obj.noises.accelerometer];

		
			K = P_apriori*H.' * (H*P_apriori*H.' + R)^-1;
			q = q_apriori + K * dz;
			P = (eye(4) - K*H) * P_apriori;

			% -- --------- ---


			q = q.'/norm(q);
			obj.quaternion = q; 
			obj.P = P;
		end


		function q = initInternal(obj, acc)

			acc = acc/norm(acc);
			ref = obj.AccRefNormalized;
			r = dot(acc, ref) + 1;
			v1 = cross(acc, ref);
			q = [ r v1 ];
			q = q/norm(q);

			% fixedYawQuaternion = axisAngle2quatern(obj.AccRefNormalized, 45/180*pi)
			% q = quatmultiply(fixedYawQuaternion, q);

			obj.quaternion = q;
			
		end

		function q = init(obj, gyr, acc, mag)
			q = obj.initInternal(acc);
		end

	end
end
