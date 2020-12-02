function context = createContextFromLocationAndDate(location, date, coordinateSystem)

    context = struct;

    context.location = location;
    context.date = date;

    coordinateSystem = lower(coordinateSystem);
    if ~sum(strmatch(coordinateSystem, {'enu', 'ned'})) error('Unknown coordinate system'); end

    qRotENUToNED = dcm2quat(roty(180) * rotz(90));
    qRotNEDToENU = quatinv(qRotENUToNED);

    % Magnetic field context
    [context.magnetic.vector, ~, context.magnetic.declination, ~, context.magnetic.magnitude] = ...
        wrldmagm(location.altitude, location.latitude, location.longitude, ...
        decyear(date.year, date.month, date.day));

    % Transform nanoTesla to microTesla
    context.magnetic.magnitude = context.magnetic.magnitude / 1000;
    context.magnetic.vector = context.magnetic.vector' / 1000;

    if strcmp(coordinateSystem, 'enu')
        context.magnetic.vector = quatrotate(qRotNEDToENU, context.magnetic.vector);
        context.magnetic.declination = -context.magnetic.declination;
    end

    % Gravity context
    context.gravity.magnitude = wrldaccm(location.latitude, location.altitude);
    context.gravity.vector = [0 0 -context.gravity.magnitude];

    if strcmp(coordinateSystem, 'enu')
        context.gravity.vector = quatrotate(qRotNEDToENU, context.gravity.vector);
    end

end
