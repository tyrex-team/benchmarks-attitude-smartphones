% This algorithm comes from paper:
%
% Design and implementation of a low-cost observer-based attitude and heading reference system
% P. Martin and E. Salaun
% Control Engineering Practice, vol. 18, no. 7, pp. 712–722, 2010
% http://www.sciencedirect.com/science/article/pii/S0967066110000201
%
% It has been implemented by sensbio@inria.fr (https://github.com/sensbio/sensbiotk)
% 	and ported in matlab/modfied by T. Michel. 
%
% This work is a part of project "On Attitude Estimation with Smartphones" 
% http://tyrex.inria.fr/mobile/benchmarks-attitude
%
% Contact :
% Thibaud Michel
% thibaud.michel@gmail.com

% TODO: Understand why it's not working in the same way for NED and ENU.
classdef QMartin < AttitudeFilter

	properties (Access = public)
		la = 0.7;
		lc = 0.1;
		ld = 0.01;
		n = 0.01;
		o = 0.01;
		k = 10;
		sigma = 0.002;
	end

	properties (Access = private)

		% Gravity scale factor
		a_s = 1;

		% Magnetic field scale factor
		c_s = 1;

		% Gyro bias
		wb = [0.0, 0.0, 0.0];

		% Quaternion director
		A; C; D;

	end


	methods (Access = public)

		function notifyReferenceVectorChanged(obj)
			notifyReferenceVectorChanged@AttitudeFilter(obj);

			A = obj.AccRef;
			A(abs(A)>0) = A(abs(A)>0)./abs(A(abs(A)>0));
			B = obj.MagRef;
			B(abs(B)>0) = B(abs(B)>0)./abs(B(abs(B)>0));
			C = cross(A, B);
			D = cross(C, A);

			obj.A = A;
			obj.C = C;
			obj.D = D;
		end

		function q = init(obj, gyr, acc, mag)

			% Quaternions construction
			acc = [0 acc];
			mag = [0 mag];
			yc = quatmultiply(acc, mag);

			% Normalization
			acc = acc / norm(acc);
			mag = mag / norm(mag);
			yc(1) = 0;
			yc = yc / norm(yc);
			
			if acc(4) == 1
				q = 1;
				qinv = 1;
			else
				qinv = [-acc(3) 1-acc(4) 0 acc(2)];
				qinv = qinv / norm(qinv);
				q = quatconj(qinv);
			end

			yc = quatmultiply(quatmultiply(q, yc), qinv);

			if yc(3) ~= 1
				qinv = quatmultiply(qinv, [-yc(2), 0, yc(4), 1 - yc(3)]);
				qinv = qinv / norm(qinv);
				q = quatconj(qinv);
			end

			obj.quaternion = q;
		end


		function q = update(obj, gyr, acc, mag, dT)
			
			q = obj.quaternion;
			qinv = quatinv(q);

			% acc = acc/norm(acc);
			% mag = mag/norm(mag);

			% Compute quaternions products
			c = cross(acc, mag); % yc = ya * yb
			d = cross(c, acc); % yd = yc * yb

			% Compute errors
			EA = obj.A - quatrotate(qinv, acc) / obj.a_s;
			EC = obj.C - quatrotate(qinv, c) / obj.c_s;
			ED = obj.D - quatrotate(qinv, d) / (obj.c_s * obj.a_s);

			LE = cross(obj.A, EA) * obj.la + cross(obj.C, EC) * obj.lc + cross(obj.D, ED) * obj.ld;

			qdot = 0.5 * quatmultiply(q, [0 gyr-obj.wb]) + quatmultiply([0 LE], q);

			sEA = norm(EA) - EA(3); % sEA = <EA, EA - A> = ||EA||² - <EA, A>
			sEC = norm(EC) - EC(2); % sEC = <EC, EC - C> = ||EC||² - <EC, C>
			sED = norm(ED) - ED(1); % sED = <ED, ED - D> = ||ED||² - <ED, D>

			NE = obj.n * (obj.la * sEA + obj.ld * sED) / (obj.la + obj.ld) ;
			OE = obj.o * (obj.lc * sEC + obj.ld * sED) / (obj.lc + obj.ld) ;


			asdot = obj.a_s * NE;
			csdot = obj.c_s * OE; 

			ME = LE * (-obj.sigma);
			wbdot = quatrotate(q, ME);

			% Integration
			q = q + qdot * dT;
			obj.wb = obj.wb + wbdot * dT;
			obj.a_s = obj.a_s + asdot * dT;
			obj.c_s = obj.c_s + csdot * dT;

			q = q / norm(q); % normalise quaternion

			obj.quaternion = q;
		end

	end
end