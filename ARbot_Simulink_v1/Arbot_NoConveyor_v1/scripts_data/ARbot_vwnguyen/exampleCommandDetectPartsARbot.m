function exampleCommandDetectPartsARbot(ARbot)
%#codegen
    resizedImg = zeros(224,224,3) 
    bboxes = 1; 
    labels = 1;
    coder.varsize('bboxes',[50,4])
    coder.varsize('labels',[50,1])
    
    % figure;
    resizedImg = imresize(ARbot.img,[224 224]);
    imshow(resizedImg);
    [bboxes,labels] = detect(ARbot.DetectorModel,resizedImg);
    
    if ~isempty(labels)
        labeledImg = insertObjectAnnotation(resizedImg,'Rectangle',bboxes,cellstr(labels));
        imshow(labeledImg);
    end
    
    ARbot.Bboxes = bboxes;
    ARbot.Labels = labels;
end
 
 