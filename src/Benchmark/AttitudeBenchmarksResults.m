classdef AttitudeBenchmarksResults < handle

	properties(Access = public)

		results = {};

		displayOrder = struct;
		displaySelection = struct;

		defaultSamplingRate;
		defaultCalibration;
	end



	methods(Access = public)

		function obj = AttitudeBenchmarksResults(obj)

			obj.displayOrder.motions = {'ar', 'texting', 'phoning', 'frontpocket', 'backpocket', 'swinging', ...
					'runningpocket', 'runninghand'};
			obj.displayOrder.devices = {'iPhone4S', 'iPhone5', 'Nexus5'};
			obj.displayOrder.sampling = [100 40 10 2];
			obj.displayOrder.calibrations = { 	{'raw', 'raw', 'raw'},	
												{'own', 'raw', 'raw'},	
												{'own', 'raw', 'own'},
												{'own', 'own', 'raw'},	
												{'own', 'own', 'own'}, 
												{'own', 'osO', 'own'},
												{'osR', 'osR', 'osR'}};				

			obj.displaySelection.bias = {'Madgwick', 'MadgwickB', 'Mahony', 'MahonyB', 'Renaudin', 'RenaudinB', 'RenaudinBG'};
			obj.displaySelection.kalman = {'Choukroun', 'ChoukrounSn', 'Ekf', 'EkfSn', 'EkfRev', 'EkfRevSn', ...
							'MichelEkf', 'MichelEkfSn', 'Renaudin', 'RenaudinSn'};

			obj.defaultSamplingRate = 100;
			obj.defaultCalibration = {'own', 'own', 'own'};
		end


		function add(obj, result);
			obj.results{end+1} = result;
		end

		function remove(obj)

			for i=1:length(obj.results)

				if isempty(obj.results{i}), continue; end

				algorithm = obj.results{i}.algorithm;
				calibration = obj.results{i}.calibration;
				user = obj.results{i}.user;
				device = obj.results{i}.device;
				disturbances = obj.results{i}.disturbances;
				motion = obj.results{i}.motion;
				sampling = obj.results{i}.sampling;

				% eg. if you want to remove all results processed by Ekf algorithm.
				if strcmp(algorithm, 'Ekf')
					obj.results{i} = {};
				end

			end

			disp([num2str(sum(cellfun('isempty',obj.results))) ' removed'])

			obj.results = obj.results(~cellfun('isempty',obj.results));
		end


		function result = get(obj, user, device, disturbances, motion, algorithm, calibration, sampling)
			result = 0;
			for i=1:length(obj.results)
				r2 = obj.results{i};
				exists = strcmp(user, r2.user) && strcmp(device, r2.device) && ...
					disturbances == r2.disturbances && strcmp(motion, r2.motion) && ...
					strcmp(r2.algorithm, algorithm) && isequal(r2.calibration, calibration) && ...
					r2.sampling == sampling;
				if exists
					result = r2;
					return;
				end
			end

		end


		function count = countAllQuaternionsCompared(obj)

			count = 0;

			for i=1:length(obj.results)
				count = count + size(obj.results{i}.errors, 1);				
			end

		end


		% columnType: calibration, motions (default), motionsmag, mag, devices, sampling, processingTime
		% displayFormat: none, simple (default), html
		function scores = compare(obj, columnsType, displayFormat)
			
			scores = struct;

			if ~exist('columnsType'), columnsType = 'motions'; end
			if ~exist('displayFormat'), displayFormat = 'simple'; end

			for i=1:length(obj.results)
				result = obj.results{i};


				switch columnsType

				case 'calibration'
					if result.sampling ~= obj.defaultSamplingRate || strcmp(lower(result.algorithm), 'os') 
						continue;
					end
					columnString = sprintf('%s%s%s', result.calibration{:});


				case 'motions'
					if result.disturbances || ~strcmp(lower(result.algorithm), 'os') && ...
						(result.sampling ~= obj.defaultSamplingRate || ~isequal(result.calibration, obj.defaultCalibration))
						continue;
					end
					columnString = result.motion;


				case 'motionsmag'
					if ~result.disturbances || ~strcmp(lower(result.algorithm), 'os') && ...
						(result.sampling ~= obj.defaultSamplingRate || ~isequal(result.calibration, obj.defaultCalibration))
						continue;
					end
					columnString = result.motion;


				case 'mag'
					if strcmp(result.motion, 'runninghand') || strcmp(result.motion, 'runningpocket') || ...
					 	~strcmp(lower(result.algorithm), 'os') && ...
						(result.sampling ~= obj.defaultSamplingRate || ~isequal(result.calibration, obj.defaultCalibration))
						continue;
					end
					columnString = sprintf('d%d', result.disturbances);


				case 'devices'
					if result.sampling ~= obj.defaultSamplingRate || ...
						strcmp(lower(result.algorithm), 'os') ~= 1 && ~isequal(result.calibration, obj.defaultCalibration)
						continue;
					end
					columnString = result.device;


				case 'sampling'
					if strcmp(lower(result.algorithm), 'os') || ~isequal(result.calibration, obj.defaultCalibration)
						continue;
					end
					columnString = sprintf('s%d', result.sampling);

				case 'bias'
					if result.disturbances || result.sampling ~= obj.defaultSamplingRate ||  ...
						~any(strcmp(result.algorithm, obj.displaySelection.bias)) || ...
						~isequal(result.calibration, obj.defaultCalibration)
						continue;
					end
					columnString = result.motion;

				case 'kalman'
					if result.disturbances || result.sampling ~= obj.defaultSamplingRate ||  ...
						~any(strcmp(result.algorithm, obj.displaySelection.kalman)) || ...
						~isequal(result.calibration, obj.defaultCalibration)
						continue;
					end
					columnString = result.motion;

				case 'processingTime'
					if strcmp(lower(result.algorithm), 'os'), continue; end

				otherwise
					error('Unknown columnsType');
				end

				
				if ~isfield(scores, sprintf('%s', result.algorithm))
					scores.(result.algorithm) = [];
				end

				if strcmp(columnsType, 'processingTime')
					scores.(result.algorithm)(end+1) = result.processingTime/result.processingQuaternions;
					continue;
				end

				if ~isfield(scores.(result.algorithm), columnString)
					scores.(result.algorithm).(columnString) = [];
				end

				[c firstIndexAfter5Sec] = min(abs(result.errors(:, 1) - 5));
				scores.(result.algorithm).(columnString)(end+1, :) = mean(result.errors(firstIndexAfter5Sec:end,2:end));
			end

			if strcmp(columnsType, 'processingTime')
				fields = fieldnames(scores);
				for i = 1:numel(fields), scores.(fields{i}) = mean(scores.(fields{i})); end;
			end

			if strcmp(displayFormat, 'none') || isempty(fieldnames(scores)) return; end

			scores = obj.formatResults(scores, columnsType);

			if strcmp(columnsType, 'processingTime')

				switch displayFormat
					case 'simple', obj.simpleDispProcessingTime(scores);
					case 'html', obj.exportProcessingTimeToHtml(scores);
					otherwise error('Not a known displayFormat');
				end
			else
				switch displayFormat
					case 'simple', obj.simpleDispScores(scores, columnsType);
					case 'html', obj.exportScoresToHtml(scores);
					otherwise error('Not a known displayFormat');
				end
			end

		end


		function scores = formatResults(obj, scores, styleFields)
			
			fields = fieldnames(scores);
			
			switch styleFields
				case 'calibration'
					orderFields = cellfun(@(S) {[S{1} S{2} S{3}]}, obj.displayOrder.calibrations);
				case 'motions'
					orderFields = obj.displayOrder.motions;
				case 'devices'
					orderFields = obj.displayOrder.devices;
				case 'sampling'
					orderFields = arrayfun(@(S) {['s' num2str(S)]}, obj.displayOrder.sampling);
				case 'bias'
					orderFields = obj.displayOrder.motions;
				case 'kalman'
					orderFields = obj.displayOrder.motions;
				case 'processingTime'
					minValue = realmax;
					for i = 1:numel(fields), minValue = min(minValue, scores.(fields{i})); end;
					for i = 1:numel(fields), scores.(fields{i}) = scores.(fields{i})/minValue; end;
					newVec = zeros(numel(fields),1); 
					for i = 1:numel(fields), newVec(i) = scores.(fields{i}); end;
					[~, I] = sort(newVec);
					orderAlgorithms = I;
				
				otherwise
					styleFields = '';
			end

			% Sort by algorithm
			if exist('orderAlgorithms')
				scores = orderfields(scores, orderAlgorithms);
			else
				scores = orderfields(scores);
			end

			if ~exist('styleFields') || isempty(styleFields) || strcmp(styleFields, 'processingTime'), return; end

			% Sort by subfield
			if exist('orderFields')
				for i = 1:numel(fields)
					subfields = scores.(fields{i});
					scores.(fields{i}) = orderfields(subfields,  ...
						intersect(orderFields, fieldnames(subfields), 'stable'));
				end
			end
		end



		function simpleDispScores(obj, scores, columnsType)

			fields = fieldnames(scores);

			maxSizeOfAlgorithmeName = max(cellfun(@length, fields));
			maxSizeOfField2 = max(cellfun(@length, fieldnames(scores.(fields{1}))));

			if strcmp(columnsType, 'sampling') maxSizeOfField2 = maxSizeOfField2 + 2; end
			if strcmp(columnsType, 'calibration') maxSizeOfField2 = maxSizeOfField2 + 8; end
			if strcmp(columnsType, 'mag') maxSizeOfField2 = length('Without Perturbations'); end

			for i = 1:numel(fields)

				subFields = scores.(fields{i});
				subFieldsNames = fieldnames(subFields);

				switch columnsType
				case 'sampling'
					subFieldsDispNames = cellfun(@(S) {[S(2:end) 'Hz']}, subFieldsNames);
				case 'mag'
					subFieldsDispNames = subFieldsNames;
					subFieldsDispNames(strcmp('d1', subFieldsDispNames)) = {'With Perturbations'};
					subFieldsDispNames(strcmp('d0', subFieldsDispNames)) = {'Without Perturbations'};
				case 'calibration'
					subFieldsDispNames = cellfun(@(S) {[S(1:3) 'Mag ' S(4:6) 'Gyr ' S(7:9) 'Acc']}, subFieldsNames);
				otherwise
					subFieldsDispNames = subFieldsNames;
				end

				
				for j = 1:numel(subFieldsNames)
					m = mean(subFields.(subFieldsNames{j}).', 2);
					disp(sprintf(['%-' num2str(maxSizeOfAlgorithmeName) 's %-' num2str(maxSizeOfField2) 's %5.1f%c ' ...
						' YPR: %5.1f%c %4.1f%c %4.1f%c'], fields{i}, subFieldsDispNames{j}, ...
						 m(1), char(176), m(2), char(176), m(3), char(176), m(4), char(176)));
				end
			end
		end



		function exportScoresToHtml(obj, scores)

			disp('<tbody>');

			fields = fieldnames(scores);
			for i = 1:numel(fields)

				disp('	<tr>');
				disp(sprintf('		<td class="algoname">%s</td>', fields{i}));

				fields2 = fieldnames(scores.(fields{i}));
				for j = 1:numel(fields2)

					m = mean(scores.(fields{i}).(fields2{j}).', 2);
					disp(sprintf('		<td><div data-html="true" data-toggle="tooltip" title="Yaw: %.1f&deg;<br />Pitch: %.1f&deg;<br />Roll: %.1f&deg;">%.1f&deg;</div></td>', m([2 3 4 1])));
				end

				disp('	</tr>');
			end

			disp('</tbody>')
		end


		function simpleDispProcessingTime(obj, scores)
			fields = fieldnames(scores);
			for i = 1:numel(fields)
				disp(sprintf('%s, score: %.1f', fields{i}, mean(scores.(fields{i}))));
			end
		end

		function exportProcessingTimeToHtml(obj, scores)
			fields = fieldnames(scores);
			for i = 1:numel(fields)
				disp(sprintf('[''%s'', %0.1f], ', fields{i}, scores.(fields{i})));
			end
		end


	end	

end


