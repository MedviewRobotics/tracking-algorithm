function rgb = plotVideo(frameLeft,bboxLeft,centroidLeft,...
k,point3d_1,point3d_2,point3d_3)

rgb = insertShape(frameLeft,'rectangle',bboxLeft(1,:),'Color','black',...
    'LineWidth',3);
rgb = insertShape(rgb,'rectangle',bboxLeft(2,:),'Color','black',...
    'LineWidth',3);
rgb = insertShape(rgb,'rectangle',bboxLeft(3,:),'Color','black',...
    'LineWidth',3);
rgb = insertText(rgb,centroidLeft(1,:) - 20,['X: ' num2str(round(point3d_1(1,k)),'%d')...
    ' Y: ' num2str(round(point3d_1(2,k)),'%d') ' Z: ' num2str(round(point3d_1(3,k)))],'FontSize',18);
rgb = insertText(rgb,centroidLeft(2,:) + 20,['X: ' num2str(round(point3d_2(1,k)),'%d')...
    ' Y: ' num2str(round(point3d_2(2,k)),'%d') ' Z: ' num2str(round(point3d_2(3,k)))],'FontSize',18);
rgb = insertText(rgb,centroidLeft(3,:) - 50,['X: ' num2str(round(point3d_3(1,k)),'%d')...
    ' Y: ' num2str(round(point3d_3(2,k)),'%d') ' Z: ' num2str(round(point3d_3(3,k)))],'FontSize',18);


end

