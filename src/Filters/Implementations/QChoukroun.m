% This algorithm comes from paper:
%
% Novel Quaternion Kalman Filter
% D. Choukroun, I. Y. Bar-Itzhack, Y. Oshman
% IEEE Transactions on Aerospace and Electronic Systems Vol. 42, No. 1 January 2006
% http://oshman.technion.ac.il/files/2016/04/Choukroun_Baritz_Oshman_T-AES_V42_No1_Jan2006.pdf
%
% It has been implemented by A. Makni and modified by T. Michel.
%
% This work is a part of project "On Attitude Estimation with Smartphones" 
% http://tyrex.inria.fr/mobile/benchmarks-attitude
%
% Contact :
% Thibaud Michel
% thibaud.michel@gmail.com

classdef QChoukroun < LinearKalmanFilter
	
	methods(Access = public)	

		function q = update(obj, gyr, acc, mag, dT)

			q = obj.quaternion.';


			acc = acc / norm(acc);
			mag = mag / norm(mag);


			s1 = 0.5 * (mag + obj.MagRefNormalized);
			d1 = 0.5 * (mag - obj.MagRefNormalized);
			H1 = [0 -d1 ; d1.' -skew(s1)];

			s2 = 0.5 * (acc + obj.AccRefNormalized);
			d2 = 0.5 * (acc - obj.AccRefNormalized);
			H2 = [0 -d2 ; d2.' -skew(s2)];
			H_bar = [H1 ; H2];


			omega_g = [0 -gyr ; gyr.' -skew(gyr)];
			phi = expm(omega_g * dT * 0.5);
			xm = phi * q; 


			E = [-q(2:4)' ; skew(q(2:4)) + q(1) * eye(3)];

			Qf = dT^2 * 0.25 * (E * obj.noises.gyroscope * E.');
			Pm = phi * obj.P * phi.' + Qf; 


			E1 = [-xm(2:4)' ; skew(xm(2:4)) + xm(1) * eye(3)]; 


			Ra = 0.25 * E1 * obj.noises.accelerometer * E1.' + 0.000001 * eye(4);
			Rmg = 0.25 * E1 * obj.noises.magnetometer * E1.' + 0.000001 * eye(4);

			R_bar = [Rmg zeros(4,4) ; zeros(4,4) Ra];
			
			S = (H_bar * Pm * H_bar.' + R_bar);

			K = Pm * H_bar.' * S^(-1);

			q = xm - K * H_bar * xm;
			q = q / norm(q);


			obj.P = (eye(4) - K * H_bar) * Pm * (eye(4) - K * H_bar).' + K * R_bar * K.';

			q = q.';
			obj.quaternion = q;  
		end
	end 
end
