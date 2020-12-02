% Align two datasets with their time
% datasetReference is a matrix of [timestamp1 value11 value12 ... value1X \n timestamp1 value21 value22 ... value2X]
% datasetInput is a matrix of [timestamp1 value11 value12 ... value1Y \n timestamp1 value21 value22 ... value2Y]
function [datasetOutput] = matchDataSetTimestamp(datasetReference, datasetInput, defaultValue)

    datasetInputLength = size(datasetInput, 1);
    datasetInputColumns = size(datasetInput, 2);

    datasetReferenceLength = size(datasetReference, 1);

    datasetOutput = zeros(datasetReferenceLength, datasetInputColumns);
    datasetOutputColumns = datasetInputColumns;

    if ~isequal(size(defaultValue), [1 datasetOutputColumns - 1])
        error('Default value not have the good size');
    end

    datasetInputPointer = 0;

    for i = 1:datasetReferenceLength
        timestampReference = datasetReference(i, 1);

        while (datasetInputPointer < datasetInputLength && ...
                datasetInput(datasetInputPointer + 1, 1) <= timestampReference)

            datasetInputPointer = datasetInputPointer + 1;
        end

        if datasetInputPointer == 0
            datasetOutput(i, :) = [timestampReference defaultValue];
        else
            datasetOutput(i, :) = [timestampReference datasetInput(datasetInputPointer, 2:end)];
        end

    end

end
