% This algorithm comes from paper:
%
% A nonlinear filtering approach for the attitude and dynamic body acceleration estimation based on inertial and magnetic sensors: Bio-logging application,
% H. Fourati, N. Manamanni, L. Afilal, Y. Handrich
% IEEE Sensors Journal, VOL. 11, NO. 1, January 2011
% https://hal.archives-ouvertes.fr/hal-00624142/
%
% and
%
% Heterogeneous Data Fusion Algorithm for Pedestrian Navigation via Foot-Mounted Inertial Measurement Unit and Complementary Filter
% H. Fourati
% IEEE Transactions on Instrumentation and Measurement, Institute of Electrical and Electronics Engineers, 2015, 64 (1), pp.221-229
% https://hal.inria.fr/hal-00999073
%
% It has been implemented by H. Fourati and modified by T. Michel.
%
% This work is a part of project "On Attitude Estimation with Smartphones" 
% http://tyrex.inria.fr/mobile/benchmarks-attitude
%
% Contact :
% Thibaud Michel
% thibaud.michel@gmail.com

classdef QFourati < AttitudeFilter

    properties (Access = private)
		Beta = 0.3;
		Ka = 2;
		Km = 1;
	end
    
    methods(Access = public)

		function q = update(obj, gyr, acc, mag, dT)

			q = obj.quaternion;	

			Km = obj.Km;
			Ka = obj.Ka;

			acc = acc/norm(acc);
			mag = mag/norm(mag);

            estimate_A = quatrotate(q, obj.AccRefNormalized);
            estimate_M = quatrotate(q, obj.MagRefNormalized);


			Measure = [acc  mag];
			Estimate = [estimate_A  estimate_M];
			delta = 2 * [Ka * skew(estimate_A) ; Km * skew(estimate_M)]';
			

			% Gradient decent algorithm corrective step
			dq = (Measure - Estimate) * ((delta * delta' + 1e-5 * eye(3))^-1 * delta)';

			qDot = 0.5 * quatmultiply(q, [0 gyr]) + obj.Beta * quatmultiply(q, [0 dq]);
			q = q + qDot * dT;
			q = q/norm(q);

			obj.quaternion = q;
		end

		
    end 
end
