function save_fig(Test)

    Test  =flag.test;
    if Test == 1 % 1 --> STEER ramp test
        FolderName = 'figures/SteerRamp';   % Your destination folder
    else
        FolderName = 'figures/SpeedRamp';   % Your destination folder
    end
    
    
    FigList = findobj(allchild(0), 'flat', 'Type', 'figure');
    for iFig = 1:length(FigList)
      FigHandle = FigList(iFig);
      % FigName = FigHandle.Name;
      FigName = replace(FigHandle.Name,' ' , '_');
      % FigName   = ['Fig' num2str(iFig)];
      saveas(FigHandle, fullfile(FolderName, [FigName '.svg']));
      saveas(FigHandle, fullfile(FolderName, [FigName '.eps']));
    %   saveas(FigHandle,filename,formattype)
    end

end