syms vx vy vz qw qx qy qz

removeFirstValue = @(v) v(2:end);



% ------ Smartphone -> Earth ------


% First 4 jacobians are based on: q * [0 vx vy vz] * q^-1

fSE1(vx, vy, vz, qw, qx, qy, qz) = removeFirstValue(quatmultiply(quatmultiply([qw qx qy qz], [0 vx vy vz]), quatinv([qw qx qy qz])));
jSE1 = jacobian(fSE1, [qw qx qy qz]);

fSE2(vx, vy, vz, qw, qx, qy, qz) = (quat2dcm(quatinv([qw qx qy qz]), 'long') * [vx vy vz].').';
jSE2 = jacobian(fSE2, [qw qx qy qz]);

fSE3(vx, vy, vz, qw, qx, qy, qz) = quatrotate(quatinv([qw qx qy qz]), [vx vy vz], 'long');
jSE3 = jacobian(fSE3, [qw qx qy qz]);

% Renaudin's form in : Quaternion based heading estimation with handheld MEMS in 
% 	indoor environments, V Renaudin, C Combettes, F Peyret
% 	2014 IEEE/ION Position, Location and Navigation Symposium-PLANS 2014, 645-656
r = skew([qx qy qz])*[vx vy vz].' + qw*[vx vy vz].';
jSE4(vx, vy, vz, qw, qx, qy, qz) = 2 * [ r [qx qy qz]*[vx vy vz].' * eye(3) - skew(r)];

myJSELong(vx, vy, vz, qw, qx, qy, qz) = jacobianSE([qw qx qy qz], [vx vy vz], 'long');

assert(isequal(jSE1, jSE2, jSE3, jSE4, myJSELong));



% Next 3 jacobians are based on:  quat2dcm(q) * v.'

fSE5(vx, vy, vz, qw, qx, qy, qz) = quat2dcm(quatinv([qw qx qy qz]), 'short') * [vx vy vz].';
jSE5 = jacobian(fSE5, [qw qx qy qz]);

fSE6(vx, vy, vz, qw, qx, qy, qz) = quatrotate(quatinv([qw qx qy qz]), [vx vy vz], 'short');
jSE6 = jacobian(fSE6, [qw qx qy qz]);

myJSEShort(vx, vy, vz, qw, qx, qy, qz) = jacobianSE([qw qx qy qz], [vx vy vz], 'short');

assert(isequal(jSE5, jSE6, myJSEShort));






% ------ Earth -> Smartphone ------

% First 4 jacobians are based on: q^-1 * [0 vx vy vz] * q

fES1(vx, vy, vz, qw, qx, qy, qz) = removeFirstValue(quatmultiply(quatmultiply(quatinv([qw qx qy qz]), [0 vx vy vz]), [qw qx qy qz]));
jES1 = jacobian(fES1, [qw qx qy qz]);

fES2(vx, vy, vz, qw, qx, qy, qz) = (quat2dcm([qw qx qy qz], 'long') * [vx vy vz].').';
jES2 = jacobian(fES2, [qw qx qy qz]);

fES3(vx, vy, vz, qw, qx, qy, qz) = quatrotate([qw qx qy qz], [vx vy vz], 'long');
jES3 = jacobian(fES3, [qw qx qy qz]);

% Renaudin's form in : Quaternion based heading estimation with handheld MEMS in 
% 	indoor environments, V Renaudin, C Combettes, F Peyret
% 	2014 IEEE/ION Position, Location and Navigation Symposium-PLANS 2014, 645-656
r = skew([qx qy qz])*[vx vy vz].' - qw*[vx vy vz].';
jES4(vx, vy, vz, qw, qx, qy, qz) = 2 * [ -r [qx qy qz]*[vx vy vz].' * eye(3) - skew(r)];

myJESLong(vx, vy, vz, qw, qx, qy, qz) = jacobianES([qw qx qy qz], [vx vy vz], 'long');


assert(isequal(jES1, jES2, jES3, jES4, myJESLong));



% Next 3 jacobians are based on:  quat2dcm(q) * v.'

% This one is different because it is based on property of unit quaternion in quat2dcm()
fES5(vx, vy, vz, qw, qx, qy, qz) = (quat2dcm([qw qx qy qz], 'short') * [vx vy vz].').';
jES5 = jacobian(fES5, [qw qx qy qz]);

fES6(vx, vy, vz, qw, qx, qy, qz) = quatrotate([qw qx qy qz], [vx vy vz], 'short');
jES6 = jacobian(fES6, [qw qx qy qz]);

myJESShort(vx, vy, vz, qw, qx, qy, qz) = jacobianES([qw qx qy qz], [vx vy vz], 'short');

assert(isequal(jES5, jES6, myJESShort));




% Following are compact forms for acceleration reference and for magnetic field reference.

fESAccENU(qw, qx, qy, qz) = quat2dcm([qw qx qy qz], 'short') * [0 0 1].';
jESAccENU = jacobian(fESAccENU, [qw qx qy qz]);
myJESAccENU(qw, qx, qy, qz) = jacobianESAcc([qw qx qy qz], 'enu');
assert(isequal(jESAccENU, myJESAccENU));


fESAccNED(qw, qx, qy, qz) = quat2dcm([qw qx qy qz], 'short') * [0 0 -1].';
jESAccNED = jacobian(fESAccNED, [qw qx qy qz]);
myJESAccNED(qw, qx, qy, qz) = jacobianESAcc([qw qx qy qz], 'ned');
assert(isequal(jESAccNED, myJESAccNED));


fESMagENU(vy, vz, qw, qx, qy, qz) = quat2dcm([qw qx qy qz], 'short') * [0 vy vz].';
jESMagENU = jacobian(fESMagENU, [qw qx qy qz]);
myJESMagENU(vy, vz, qw, qx, qy, qz) = jacobianESMag([qw qx qy qz], vy, vz, 'enu');
assert(isequal(jESMagENU, myJESMagENU));


fESMagNED(vx, vz, qw, qx, qy, qz) = quat2dcm([qw qx qy qz], 'short') * [vx 0 vz].';
jESMagNED = jacobian(fESMagNED, [qw qx qy qz]);
myJESMagNED(vx, vz, qw, qx, qy, qz) = jacobianESMag([qw qx qy qz], vx, vz, 'ned');
assert(isequal(jESMagNED, myJESMagNED));
