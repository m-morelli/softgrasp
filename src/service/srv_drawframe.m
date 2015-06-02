function srv_drawframe(Tr, frameName, PlotOpt)
    hold on;
    ext_drawframe(Tr, PlotOpt.frameScale);
    hold on;
    plot3(Tr(1,4),Tr(2,4),Tr(3,4),'.r','MarkerSize',PlotOpt.frameOriginMarkerSize);
    fh = text(Tr(1,4),Tr(2,4),Tr(3,4),frameName,'FontName','FixedWidth');
    set(fh,'fontWeight','bold');
    set(fh,'HorizontalAlignment','left');