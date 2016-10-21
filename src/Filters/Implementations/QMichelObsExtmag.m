% This algorithm is based on QMichelObs and an external magnetic detector 
% 	has been added
%
% This algorithm has been implemented by T. Michel
%
% This work is a part of project "On Attitude Estimation with Smartphones" 
% http://tyrex.inria.fr/mobile/benchmarks-attitude
%
% Contact :
% Thibaud Michel
% thibaud.michel@gmail.com

classdef QMichelObsExtmag < AttitudeFilter

    properties (Access = private)

		MagNormThreshold = 15;

		Beta = 0.3;
		Ka = 2;
		Km = 1;

	end
    
    methods(Access = public)

		function q = update(obj, gyr, acc, mag, dT)

			magUpdate = abs( norm(mag) - obj.MagRefNorm) < obj.MagNormThreshold;

			% Use the normal filter
			q = obj.updateInternal(gyr, acc, mag, dT, magUpdate);
		end
		
    end 

    methods (Access = private)

		function q = updateInternal(obj, gyr, acc, mag, dT, magUpdate)
			
			q = obj.quaternion;

			acc = acc/norm(acc);
			mag = mag/norm(mag);

			estimate_A = quatrotate(q, obj.AccRefNormalized);
			estimate_M = quatrotate(q, obj.MagRefNormalized);
			
			Measure = [acc  mag];
			Estimate = [estimate_A  estimate_M];

			delta = 2 * [obj.Ka * skew(estimate_A) ; obj.Km * magUpdate * skew(estimate_M)]';

			% Gradient decent algorithm corrective step
			dq = (Measure - Estimate) * ((delta * delta' + 1e-5 * eye(3))^-1 * delta)';

			qDot = 0.5 * quatmultiply(q, [0 gyr]) + obj.Beta * quatmultiply(q, [0 dq]);
			q = q + qDot * dT;
			q = q/norm(q);

			obj.quaternion = q;
		end

	end
end
