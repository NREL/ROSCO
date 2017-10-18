%% saveFigs
% Saves all current figures. Can only be used after plotData is excecuted.
% Same code as in plotData, used so it can easily be executed from the
% command window.
    figArray=findall(0,'type','figure');
    for i = 1:length(figArray)
        figure(figArray(i).Number)
        saveas(figArray(i),[debugFolder 'fig' get(get(gca,'title'),'string') '.fig']);
    end
    disp(['Saved all figures to ' debugFolder(1:end-1)]);