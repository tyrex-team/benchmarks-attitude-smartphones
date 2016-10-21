% This algorithm comes from paper:
%
% Magnetic, acceleration fields and gyroscope quaternion (MAGYQ)-based attitude estimation with smartphone sensors for indoor pedestrian navigation 
% V. Renaudin, C. Combettes
% Sensors, vol. 14, no. 12, pp. 22 864–22 890, 2014
% http://www.mdpi.com/1424-8220/14/12/22864
%
% It has been implemented by T. Michel.
%
% This work is a part of project "On Attitude Estimation with Smartphones" 
% http://tyrex.inria.fr/mobile/benchmarks-attitude
%
% Contact :
% Thibaud Michel
% thibaud.michel@gmail.com

classdef QRenaudin < ExtendedKalmanFilter

	properties (Access = public)

		b_q_omega = [0 0 0 0]';

	end


	methods (Access = public)


		function q = update(obj, gyr, acc, mag, dT)
			
			q = obj.quaternion;



			stdGyr = sqrt(diag(obj.noises.gyroscope)).';
			noiseQOmega = diag([1 - cos(norm(stdGyr)/2*dT) sin(norm(stdGyr)/2*dT)*stdGyr/norm(stdGyr)].^2);



			% (1)
			if(norm(gyr) ~= 0)
				q_omega = [cos(norm(gyr)/2*dT)  sin(norm(gyr)/2*dT)*gyr/norm(gyr)]; % q_{y_g}
			else
				q_omega = [1 0 0 0];
			end

			% ---- Time update ----

			% \hat{\delta_{x}}_k^- = f(\hat{\delta_{x}}_{k-1})
			q_apriori = quatmultiply(q, q_omega); % (3)
			q_apriori = q_apriori/norm(q_apriori);
			
			F = obj.C(q_omega);
			G = -obj.M(q);

			% P_k^- = F
			P_apriori = F*obj.P*F.' + G*noiseQOmega*G.'; % (5)
			
			q_apriori_inv = [ q_apriori(1) -q_apriori(2:4)];


			% --------- Correction ------------

			acc = acc/norm(acc);
			mag = mag/norm(mag);

	
			dz = [ 	obj.MagRefNormalized - quatrotate(q_apriori_inv, mag, 'long') ...
					obj.AccRefNormalized - quatrotate(q_apriori_inv, acc, 'long')]';

			H = [	jacobianSE(q_apriori, mag, 'long')
					jacobianSE(q_apriori, acc, 'long')];
			
			R = [obj.noises.magnetometer zeros(3,3) ; zeros(3,3) obj.noises.accelerometer];

		
			% (12)
			K = P_apriori*H.' * (H*P_apriori*H.' + R)^-1;
			q = q_apriori.' + K * dz;
			P = (eye(4) - K*H) * P_apriori;

			obj.quaternion = q.'/norm(q);
			obj.P = P;

			q = obj.quaternion;
		end

	end

end