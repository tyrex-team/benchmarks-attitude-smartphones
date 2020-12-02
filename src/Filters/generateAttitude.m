% Generate quaternions about attitude estimation from sensors measurements.
%
% timestamp - timestamps of measurements [1 x N]
% acc - accelerometer measurements maxtrix [3 x N]
% gyr - gyroscope measurements matrix [3 x N]
% mag - magnetometer measurements matrix [3 x N]
%
% algorithm - algorithm to generate attitude from measurements
%	Gyro
% 	Mahony, MahonyB, MahonyMartin, MahonyMartinExtmagWtRep
% 	Madgwick, MadgwickB
% 	Fourati, FouratiExtacc, FouratiMartin, FouratiMartinExtmagWtRep
% 	Martin, MartinB, MartinEkf
% 	Choukroun, ChoukrounSn
% 	Sabatini, SabatiniExtacc, SabatiniExtmag, SabatiniExtaccExtmag
% 	MichelObs, MichelObsExtmag','MichelObsExtmagWt, MichelObsExtmagWtRep
% 	Renaudin, RenaudinSn, RenaudinB, RenaudinBG, RenaudinExtacc, RenaudinExtmag, RenaudinExtaccExtmag, RenaudinBGExtaccExtmag
% 	Ekf, EkfExp, EkfLJ, EkfSn, EkfRev, EkfRevSn, EkfWithoutMag
% 	MichelEkf, MichelEkfSn, MichelEkfExtmag, MichelEkfExtmagWt, MichelEkfExtmagWtRep
%
% context - structure with properties about the geographic and inertial context.
%	context.gravity.magnitude - magnitude of gravitational acceleration in m.s-2 (~ 9.8 m.s-2) (must be positive)
% 	context.magnetic.northComponent - component of magnetic vector in earth magnetic field pointing to north in micro-Tesla. (must be positive)
%	context.magnetic.downComponent - component of magnetic vector in earth magnetic field pointing to down in micro-Tesla. (must be positive)
%	context.noises.accelerometer - white noise of accelerometer sensor.
%	context.noises.magnetometer - white noise of magnetometer sensor.
%	context.noises.gyroscope - white noise of gyroscope sensor.
%	context.noises.gyroscopeBias - white noise of gyroscope bias sensor.
%
% coordinateSystem - ENU (East North Up), NED (North East Down)
%

function attitude = generateAttitude(timestamp, acc, gyr, mag, algorithm, context, coordinateSystem)

    if size(acc, 1) ~= size(gyr, 1) || size(acc, 1) ~= size(mag, 1) || size(acc, 1) ~= size(timestamp, 1) || ...
            size(acc, 2) ~= 3 || size(mag, 2) ~= 3 || size(gyr, 2) ~= 3 || size(timestamp, 2) ~= 1
        error('Bad input size of measurements vectors')
    end

    dt = diff(timestamp);
    dataLength = length(timestamp);
    algorithm = lower(algorithm);

    useSensorNoises = false;

    switch algorithm

        case 'gyro', filter = GyroFilter;

            % Observers
        case 'madgwick', filter = QMadgwick;
        case 'madgwickb', filter = QMadgwickB;
        case 'mahony', filter = QMahony;
        case 'mahonymartin', filter = QMahonyMartin;
        case 'mahonymartinextmagwtrep', filter = QMahonyMartinExtmagWtRep;
        case 'mahonyb', filter = QMahonyB;
        case 'fourati', filter = QFourati;
        case 'fouratiextacc', filter = QFouratiExtacc;
        case 'fouratimartin', filter = QFouratiMartin;
        case 'fouratimartinextmagwtrep', filter = QFouratiMartinExtmagWtRep;
        case 'martin', filter = QMartin;
        case 'martinb', filter = QMartinB;
        case 'martinekf', filter = QMartinEkf;
        case 'michelobs', filter = QMichelObs;
        case 'michelobsextmag', filter = QMichelObsExtmag;
        case 'michelobsextmagwt', filter = QMichelObsExtmagWt;
        case 'michelobsextmagwtrep', filter = QMichelObsExtmagWtRep;

            % EKF
        case 'ekfsn', filter = QEkf; useSensorNoises = true;
        case 'ekf', filter = QEkf;
        case 'ekfwithoutmag', filter = QEkfWithoutMag;
        case 'ekfrevsn', filter = QEkfRev; useSensorNoises = true;
        case 'ekfrev', filter = QEkfRev;
        case 'ekfexp', filter = QEkfExp;
        case 'ekflj', filter = QEkfLongJacobian;
        case 'sabatini', filter = QSabatini;
        case 'sabatiniextacc', filter = QSabatiniExtacc;
        case 'sabatiniextmag', filter = QSabatiniExtmag;
        case 'sabatiniextaccextmag', filter = QSabatiniExtaccExtmag;
        case 'renaudin', filter = QRenaudin;
        case 'renaudinsn', filter = QRenaudin; useSensorNoises = true;
        case 'renaudinb', filter = QRenaudinB;
        case 'renaudinbg', filter = QRenaudinBG;
        case 'renaudinextacc', filter = QRenaudinExtacc;
        case 'renaudinextmag', filter = QRenaudinExtmag;
        case 'renaudinextaccextmag', filter = QRenaudinExtaccExtmag;
        case 'renaudinbgextaccextmag', filter = QRenaudinBGExtaccExtmag;
        case 'michelekf', filter = QMichelEkf;
        case 'michelekfsn', filter = QMichelEkf; useSensorNoises = true;
        case 'michelekfextmag', filter = QMichelEkfExtmag;
        case 'michelekfextmagwt', filter = QMichelEkfExtmagWt;
        case 'michelekfextmagwtrep', filter = QMichelEkfExtmagWtRep;

            % LKF
        case 'choukroun', filter = QChoukroun;
        case 'choukrounsn', filter = QChoukroun; useSensorNoises = true;

        otherwise
            error(['Algorithm ', algorithm, ' is unknown']);
    end

    % Let's define MagRef and AccRef temporarly in Earth Magnetic Field frame

    qMagneticToTrue = dcm2quat(rotz(context.magnetic.declination));
    qTrueToMagnetic = quatinv(qMagneticToTrue);

    filter.MagRef = quatrotate(qTrueToMagnetic, context.magnetic.vector);
    filter.AccRef = quatrotate(qTrueToMagnetic, context.gravity.vector); % Should do nothing

    filter.MagRef(abs(filter.MagRef) < 1e-12) = 0;
    filter.AccRef(abs(filter.AccRef) < 1e-12) = 0;

    if sum(abs(filter.MagRef) > 0) ~= 2 || sum(abs(filter.AccRef) > 0) ~= 1
        error('Reference vectors are not well constructed'); % This should never happen
    end

    filter.coordinateSystem = lower(coordinateSystem);
    filter.notifyReferenceVectorChanged;

    %%% Replace default noises by sensors noises

    if useSensorNoises && isfield(context, 'noises') && isfield(context.noises, 'accelerometer')
        filter.noises.accelerometer = diag(context.noises.accelerometer);
    end

    if useSensorNoises && isfield(context, 'noises') && isfield(context.noises, 'magnetometer')
        filter.noises.magnetometer = diag(context.noises.magnetometer);
    end

    if useSensorNoises && isfield(context, 'noises') && isfield(context.noises, 'gyroscope')
        filter.noises.gyroscope = diag(context.noises.gyroscope);
    end

    if useSensorNoises && isfield(context, 'noises') && isfield(context.noises, 'gyroscopeBias')
        filter.noises.gyroscopeBias = diag(context.noises.gyroscopeBias);
    end

    %%% Generate quaternions

    attitude = zeros(dataLength, 4);
    attitude(1, :) = filter.init(gyr(1, :), acc(1, :), mag(1, :));

    for i = 2:dataLength
        attitude(i, :) = filter.update(gyr(i, :), acc(i, :), mag(i, :), dt(i - 1));
    end

    % Come back to True North frame
    attitude = quatmultiply(qTrueToMagnetic, attitude);

end
