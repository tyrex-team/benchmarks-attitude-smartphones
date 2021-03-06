% This algorithm comes from paper:
%
% Quaternion-based extended kalman filter for determining orientation by inertial and magnetic sensing
% A. Sabatini
% Biomedical Engineering, IEEE Transactions on, vol. 53, no. 7, pp. 1346–1356, 2006.
% https://www.ncbi.nlm.nih.gov/pubmed/16830938
%
% It has been implemented by T. Michel.
%
% This work is a part of project "On Attitude Estimation with Smartphones"
% http://tyrex.inria.fr/mobile/benchmarks-attitude
%
% Contact :
% Thibaud Michel
% thibaud.michel@gmail.com

classdef QSabatini < ExtendedKalmanFilter

    methods (Access = public)

        function q = update(obj, gyr, acc, mag, dT)
            q = obj.quaternion.';

            acc = acc / norm(acc);
            mag = mag / norm(mag);

            % -- Prediction ---

            F = obj.C([1 0.5 * dT * gyr]);

            q_apriori = F * q;

            E = [-q(2:4).'; skew(q(2:4)) + q(1) * eye(3)];
            Qk = (dT / 2)^2 * (E * obj.noises.gyroscope * E.');

            P_apriori = F * obj.P * F.' + Qk;

            % -- --------- ---

            % -- Correction --

            q_apriori_inv = [q_apriori(1); -q_apriori(2:4)];

            dz = [obj.MagRefNormalized - quatrotate(q_apriori_inv, mag) ...
                    obj.AccRefNormalized - quatrotate(q_apriori_inv, acc)]';

            H = [jacobianSE(q_apriori, mag)
                jacobianSE(q_apriori, acc)];

            R = [obj.noises.magnetometer zeros(3, 3); zeros(3, 3) obj.noises.accelerometer];

            K = P_apriori * H.' * (H * P_apriori * H.' + R)^ - 1;
            q = q_apriori + K * dz;
            P = (eye(4) - K * H) * P_apriori;

            % -- --------- ---

            q = q / norm(q);
            q = q.';
            obj.quaternion = q;
            obj.P = P;
        end

    end

end
