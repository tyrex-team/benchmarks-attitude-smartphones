% This algorithm comes from S. Madgwick internal report:
%
% An efficient orientation filter for inertial and inertial/magnetic sensor arrays
% Sebastian O.H. Madgwick
% 2010
% http://www.x-io.co.uk/res/doc/madgwick_internal_report.pdf
%
% It has been implemented by S.O.H. Madgwick and modified by T. Michel 
%
% This work is a part of project "On Attitude Estimation with Smartphones" 
% http://tyrex.inria.fr/mobile/benchmarks-attitude
%
% Contact :
% Thibaud Michel
% thibaud.michel@gmail.com

classdef QMadgwickB < AttitudeFilter

	properties (Access = private)
		Beta = 0.08;  
		Zeta = 0.016;

		w_b = [0 0 0];
	end

	methods (Access = public)


		function q = update(obj, gyr, acc, mag, dT)
			q = obj.quaternion; 
			
			acc = acc / norm(acc);
			mag = mag / norm(mag);

			h = quatrotate([q(1) -q(2:4)], mag);
			
			if strcmp(obj.coordinateSystem, 'enu')
				b = [0 norm([h(1) h(2)]) h(3)];
			else
				b = [norm([h(1) h(2)]) 0 h(3)];
			end

			% Gradient decent algorithm corrective step
			F = [	quatrotate(q, obj.AccRefNormalized) - acc ...
					quatrotate(q, b) - mag]';

			J = [	jacobianES(q, obj.AccRefNormalized)
					jacobianES(q, b)];


			step = (J'*F);
			step = step / norm(step);

			% Compute angular estimated direction of the gyroscope error
			w_err = 2 * [q(1) * step(2) - q(2) * step(1) - q(3) * step(4) + q(4) * step(3)
						 q(1) * step(3) + q(2) * step(4) - q(3) * step(1) - q(4) * step(2)
						 q(1) * step(4) - q(2) * step(3) + q(3) * step(2) - q(4) * step(1)];

			% Compute and remove the gyroscope biases
			obj.w_b = obj.w_b + w_err' * dT * obj.Zeta;

			w = [0 gyr - obj.w_b];

			% Compute rate of change of quaternion
			qDot = 0.5 * quatmultiply(q, w) - obj.Beta * step';

			q = q + qDot * dT;
			q = q / norm(q);

			obj.quaternion = q;
		end


	end
end