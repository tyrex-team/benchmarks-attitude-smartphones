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

classdef QRenaudinBGExtaccExtmag < ExtendedKalmanFilter

	properties (Access = private)

		b_q_omega = [0 0 0 0];

		lastMag;
		lastAcc;

		QSF_Acc_Window = 10;
		QSF_Acc_Values = [];
		QSF_Acc_Norm_Values = [];
		QSF_Acc_gamma = 3.92;
		QSF_Acc_outlier = 4.90;

		QSF_Mag_Window = 10;
		QSF_Mag_Values = [];
		QSF_Mag_Norm_Values = [];
		QSF_Mag_gamma = 6;
		QSF_Mag_gamma_Earth = 10;
		QSF_Mag_outlier = 8;
		QSF_Mag_xi = 47;
	end


	methods (Access = public)

		function obj = QRenaudinBGExtaccExtmag()

			obj.P = diag([1e-2 1e-2 1e-2 1e-2 1e-6 1e-6 1e-6 1e-6]);

		end

		function q = init(obj, gyr, acc, mag)
			q = init@AttitudeFilter(obj, gyr, acc, mag);

			obj.lastMag = mag;
			obj.lastAcc = acc;
		end


		function q = update(obj, gyr, acc, mag, dT)
			
			q = obj.quaternion;

			stdGyr = sqrt(diag(obj.noises.gyroscope)).';
			noiseQOmega = diag([1 - cos(norm(stdGyr)/2*dT) sin(norm(stdGyr)/2*dT)*stdGyr/norm(stdGyr)].^2);

			stdGyrBias = sqrt(diag(obj.noises.gyroscopeBias)).';
			noiseQOmegaBias = diag([1 - cos(norm(stdGyrBias)/2*dT) sin(norm(stdGyrBias)/2*dT)*stdGyrBias/norm(stdGyrBias)].^2);


			Q = [	noiseQOmega 	zeros(4,4) 		;
					zeros(4,4) 		noiseQOmegaBias ];


			% (1)
			if(norm(gyr) ~= 0)
				q_y_g = [cos(norm(gyr)/2*dT)  sin(norm(gyr)/2*dT)*gyr/norm(gyr)]; % q_{y_g}
			else
				q_y_g = [1 0 0 0];
			end
				
			% (2)
			b_q_omega = obj.b_q_omega;

			q_omega = q_y_g - b_q_omega; % \hat{q_{\omega}}
			% b_a = obj.b_a;

			% ---- Time update ----

			% \hat{\delta_{x}}_k^- = f(\hat{\delta_{x}}_{k-1})
			q_apriori = quatmultiply(q, q_omega); % (3)
			q_apriori = q_apriori/norm(q_apriori);
			b_q_omega_apriori = b_q_omega; % (4)


			x_apriori = [ q_apriori.' ; b_q_omega_apriori.' ];

			
			F = [	obj.C(q_omega) 	-obj.M(q)	 	;
					zeros(4,4)		eye(4,4)	];

			G = [	-obj.M(q)	zeros(4,4)	;
					zeros(4,4)	eye(4)*dT 	];


			% P_k^- = F
			P_apriori = F*obj.P*F.' + G*Q*G.'; % (5)

			q_apriori_inv = [ q_apriori(1)  -q_apriori(2:4)];
			q_omega_inv = [ q_omega(1)  -q_omega(2:4)];


			% --------- QSF ------------
			% QSF Mag
			normMag = norm(mag);

			if(length(obj.QSF_Mag_Norm_Values) >= obj.QSF_Mag_Window)
				obj.QSF_Mag_Values(1,:) = [];
				obj.QSF_Mag_Norm_Values(1) = [];
			end
			obj.QSF_Mag_Values(end+1, :) = quatrotate(q_apriori_inv, mag); % Add new value to the window
			obj.QSF_Mag_Norm_Values(end+1) = normMag; % Add new value to the window


			if var(obj.QSF_Mag_Norm_Values) < obj.QSF_Mag_gamma && ...
				abs(normMag - mean(obj.QSF_Mag_Norm_Values)) <= obj.QSF_Mag_outlier
				QSF_Mag = true;
				obj.MagRef = mean(obj.QSF_Mag_Values);
				obj.MagRef = obj.MagRef/norm(obj.MagRef);
			else
				QSF_Mag = false;
			end

		

			% --------- QSF ------------
			% QSF Acc
			normAcc = norm(acc);

			if(length(obj.QSF_Acc_Norm_Values) >= obj.QSF_Acc_Window)
				obj.QSF_Acc_Norm_Values(1) = [];
			end
			obj.QSF_Acc_Norm_Values(end+1) = normAcc; % Add new value to the window

			if length(obj.QSF_Acc_Norm_Values) == obj.QSF_Acc_Window
				QSF_Acc = var(obj.QSF_Acc_Norm_Values) < obj.QSF_Acc_gamma;
			else
				QSF_Acc = false;
			end


			% --------- Correction ------------

			acc = acc/norm(acc);
			mag = mag/norm(mag);

			% Magnetometer
			if QSF_Mag

				dz_qsf_mag = obj.MagRefNormalized - quatrotate(q_apriori_inv, mag, 'long');
				H_mag = [ jacobianSE(q_apriori, mag, 'long') zeros(3,4) ];

				dz_grad_mag = mag - quatrotate(q_omega, obj.lastMag, 'long');
				H_mag = [H_mag ; zeros(3,4) obj.h3(q_omega, mag)];

				dz_mag = [dz_qsf_mag.' ; dz_grad_mag.'];
			end


			% Accelerometer
			if QSF_Acc

				dz_qsf_acc = obj.AccRefNormalized - quatrotate(q_apriori_inv, acc, 'long');
				H_acc = [ jacobianSE(q_apriori, acc, 'long') zeros(3,4)];	

				dz_grad_acc = acc - quatrotate(q_omega, obj.lastAcc, 'long');
				H_acc = [H_acc ; zeros(3,4)	obj.h3(q_omega, acc)];

				dz_acc = [dz_qsf_acc.' ; dz_grad_acc.'];
			end

			if QSF_Acc && QSF_Mag
				
				H = [H_mag ; H_acc];
				dz = [dz_mag ; dz_acc];
				
				R = zeros(12, 12);
				R(1:3, 1:3) = obj.noises.magnetometer;
				R(4:6, 4:6) = obj.noises.magnetometer;
				R(7:9, 7:9) = obj.noises.accelerometer;
				R(10:12, 10:12) = obj.noises.accelerometer;
				

			elseif QSF_Mag
				H = H_mag;
				dz = dz_mag;
				R = [ obj.noises.magnetometer zeros(3,3); zeros(3,3) obj.noises.magnetometer ]; 
			elseif QSF_Acc
				H = H_acc;
				dz = dz_acc;
				R = [ obj.noises.accelerometer zeros(3,3); zeros(3,3) obj.noises.accelerometer ]; 
			end


			if QSF_Acc | QSF_Mag
				% (12)
				K = P_apriori*H.' * (H*P_apriori*H.' + R)^-1;

				% (13)
				x = x_apriori + K * dz;
				P = (eye(length(x)) - K*H) * P_apriori;
			else
				x = x_apriori;
				P = P_apriori;
			end

			obj.quaternion = x(1:4).'/norm(x(1:4));
			obj.b_q_omega = x(5:8).';
			obj.P = P;

			obj.lastMag = mag;
			obj.lastAcc = acc;

			q = obj.quaternion;
		end

	end
end