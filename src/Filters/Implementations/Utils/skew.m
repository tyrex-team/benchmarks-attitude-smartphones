function R = skew(w)
% Generates a skew-symmetric matrix given a vector w

  if isnumeric(w),
    R = zeros(3,3);
  end
  
  R(1,2) = -w(3);
  R(1,3) =  w(2);
  R(2,3) = -w(1);

  R(2,1) =  w(3);
  R(3,1) = -w(2);
  R(3,2) =  w(1);

%   R(1,1) = 0;
%   R(2,2) = 0;
%   R(3,3) = 0;
  
end