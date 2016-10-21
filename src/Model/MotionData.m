
classdef MotionData < handle
   
  properties(Access = public)

    datasetLink

    timestamp

    position = [];
    attitude = [];

    % Sometimes qualisys system reports bad values
    badValue = [];

  end
  

  methods(Access = public)

      % Add bad values manually
      function addBadValues(obj, badValues)

          for i=1:size(badValues, 1)
              obj.badValue(obj.getRangeIndexFromTimestamps(badValues(i, 1), badValues(i, 2))) = 1;
          end

      end
  
      function range = getRangeIndexFromTimestamps(obj, startTime, endTime)

        assert(endTime > startTime, 'endTime <= startTime');

        N = length(obj.timestamp);
        
        i = 1;
        while i <= N && obj.timestamp(i) < startTime
          i = i + 1;
        end

        startIndex = i;

        while i <= N && obj.timestamp(i) < endTime
          i = i + 1;
        end

        range = startIndex:i;
      end
    end
end