classdef AttitudeFilter < handle

    properties (Access = public)

        quaternion; % Attitude

        AccRef; % Gravity vector in Earth Frame
        MagRef; % Magnetic vector in Magnetic Earth Frame

        noises;

        coordinateSystem = 'unknown'; % enu or ned

    end

    properties (Access = protected)

        AccMagAngleRef; % Angle between both vectors

        MagRefNorm;
        MagRefNormalized;

        AccRefNorm;
        AccRefNormalized;

    end

    methods (Abstract)
        quaternion = update(obj, gyr, acc, mag, dT)
    end

    methods (Access = public)

        function q = init(obj, gyr, acc, mag)

            H = cross(mag, acc);
            H = H / norm(H);

            acc = acc / norm(acc);
            M = cross(acc, H);

            switch obj.coordinateSystem
                case 'enu'
                    R = [H(1) M(1) acc(1)
                        H(2) M(2) acc(2)
                        H(3) M(3) acc(3)];
                case 'ned'
                    R = [M(1) H(1) -acc(1)
                        M(2) H(2) -acc(2)
                        M(3) H(3) -acc(3)];
                otherwise
                    error('Unknown coordinateSystem')
            end

            q = dcm2quat(R);
            obj.quaternion = q;
        end

        %% Development of a planar low cost Inertial Measurement Unit for UAVs and MAVs, Samuel Felix Fux
        %% https://www.researchgate.net/publication/236632203_Development_of_a_planar_low_cost_Inertial_Measurement_Unit_for_UAVs_and_MAVs
        % function q = init(obj, gyr, acc, mag)

        % 	acc = acc/norm(acc);
        % 	mag = mag/norm(mag);

        % 	pitch = atan2(acc(2), acc(3));
        % 	roll = asin(-acc(1));
        % 	yaw = atan2( mag(3)*sin(pitch) - mag(2)*cos(pitch), ...
        % 		mag(1)*cos(roll) + (mag(2) * sin(pitch) + mag(3) * cos(pitch)) * sin(roll));

        % 	q = angle2quat(yaw, pitch, roll);

        % 	obj.quaternion = q;
        % end

        function obj = AttitudeFilter()
            obj.quaternion = [1 0 0 0];
        end

        function notifyReferenceVectorChanged(obj)

            obj.MagRefNorm = norm(obj.MagRef);
            obj.AccRefNorm = norm(obj.AccRef);
            obj.AccMagAngleRef = atan2(norm(cross(obj.AccRef, obj.MagRef)), dot(obj.AccRef, obj.MagRef));

            obj.MagRefNormalized = obj.MagRef / obj.MagRefNorm;
            obj.AccRefNormalized = obj.AccRef / obj.AccRefNorm;

        end

        function q = gyroInt(obj, gyr, dT)

            q = obj.quaternion;

            q = quatmultiply(q, [1 0.5 * dT * gyr]);
            q = q / norm(q);

            obj.quaternion = q;

        end

    end

end
