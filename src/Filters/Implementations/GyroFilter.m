% This algorithm is a common Extended Kalman Filter
%
% It has been implemented by T. Michel
%
% This work is a part of project "On Attitude Estimation with Smartphones"
% http://tyrex.inria.fr/mobile/benchmarks-attitude
%
% Contact :
% Thibaud Michel
% thibaud.michel@gmail.com

classdef GyroFilter < AttitudeFilter

    methods (Access = public)

        function q = update(obj, gyr, acc, mag, dT)

            q = obj.quaternion;

            % q = quatmultiply(q, [1  0.5 * dT * gyr]);
            % q = q/norm(q);

            q_omega = [cos(norm(gyr) / 2 * dT) sin(norm(gyr) / 2 * dT) * gyr / norm(gyr)];
            q = quatmultiply(q, q_omega);

            obj.quaternion = q;

        end

    end

end
