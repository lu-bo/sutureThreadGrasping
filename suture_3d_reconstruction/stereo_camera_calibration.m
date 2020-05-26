function stereoParams = stereo_camera_calibration(PATH_camera_1, PATH_camera_2, pyhsical_square_length)

    leftImages = []; rightImages = [];
    
    imgDir_1  = dir([PATH_camera_1 '*.jpg']); 
    imgDir_2  = dir([PATH_camera_2 '*.jpg']);
    left_calibration_image_prefix  = {imgDir_1.folder};
    right_calibration_image_prefix = {imgDir_2.folder};
    
    left_calibration_image_name = {imgDir_1.name};
    right_calibration_image_name = {imgDir_2.name};
    
    for i = 1:length(imgDir_1)          
        leftImages.Files{i} = [left_calibration_image_prefix{i} '\' left_calibration_image_name{i}]; 
    end
    for i = 1:length(imgDir_2)          
        rightImages.Files{i} = [right_calibration_image_prefix{i} '\' right_calibration_image_name{i}]; 
    end

    
    [imagePoints,boardSize] = detectCheckerboardPoints(leftImages.Files,rightImages.Files);
    
    squareSize = pyhsical_square_length;
    worldPoints = generateCheckerboardPoints(boardSize,squareSize);
    
    I = imread(leftImages.Files{1}); 
    imageSize = [size(I,1),size(I,2)];
    stereoParams = estimateCameraParameters(imagePoints,worldPoints, ...
                                  'ImageSize',imageSize);
    
    %showReprojectionErrors(stereoParams);
end