% Thibaud Michel
% January 2014

function showMatrix(figureName, datasets, legends, titles, txlabel, tylabel)

    vectorSize = size(datasets{1}, 2) - 1;

    figure('name', figureName);
    
    col = hsv(length(datasets));

    for i = 1:vectorSize

        subplot(vectorSize, 1, i);
        hold on; 
        grid on;
        for k = 1:length(datasets)
            plot(datasets{k}(:,1), datasets{k}(:,i+1), 'color', col(k,:)); 
        end
        hold off;
        legend(legends);
        title(titles{i});

        if exist('txlabel') xlabel(txlabel); end
        if exist('tylabel') ylabel(tylabel); end
    end

end

