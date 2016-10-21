% This algorithm comes from paper:
%
% Nonlinear Complementary Filters on the Special Orthogonal Group
% R. Mahony, Tarek Hamel, Jean-Michel Pflimlin
% IEEE Transactions on, vol. 53, no. 5, pp. 1203–1218, 2008.
% https://hal.archives-ouvertes.fr/hal-00488376
%
% It has been implemented by S.O.H. Madgwick and modified by T. Michel 
%
% This work is a part of project "On Attitude Estimation with Smartphones" 
% http://tyrex.inria.fr/mobile/benchmarks-attitude
%
% Contact :
% Thibaud Michel
% thibaud.michel@gmail.com

classdef QMahonyB < AttitudeFilter

	properties (Access = public)

		Beta = 1;    
		Zeta = 0.2;  

		k1 = 1;
		k2 = 1;

		eInt = [0 0 0];
	end    
 
	methods (Access = public)

		function q = update(obj, gyr, acc, mag, dT)
			q = obj.quaternion; 

			acc = acc / norm(acc); 
 			mag = mag / norm(mag); 
 
            v = quatrotate(q, obj.AccRefNormalized);
            w = quatrotate(q, obj.MagRefNormalized);

			% Error is sum of cross product between estimated direction and measured direction of fields
			e = obj.k1 * cross(acc, v) + obj.k2 * cross(mag, w);
			obj.eInt = obj.eInt + e * dT;   

			% Apply feedback terms
			gyr = gyr + obj.Beta * e + obj.Zeta * obj.eInt;            
			
			qDot = 0.5 * quatmultiply(q, [0 gyr]);
 
			q = q + qDot * dT;
			q = q / norm(q);

			obj.quaternion = q;
		end

	end
end