function [edgecontour_pattern, contours_totoal_number, detected_contours] = DL_based_contour_segmentation(Segmented_image_name, max_row, max_col)

    % reload python function
    
    % in PC python version
    pyObj = py.importlib.import_module('contourCalculatorV2');
    
    % in laptop python version
    %pyObj = py.importlib.import_module('contourCalculator');
    
    py.importlib.reload(pyObj); % python 3
    %py.reload(pyObj) % python 2
    
    %detected_contours = py.contourCalculator.contour_function('left_image.jpg');
    
    % in PC python version
    detected_contours = py.contourCalculatorV2.contour_function(Segmented_image_name);
    
    % in laptop python version
    %detected_contours = py.contourCalculator.contour_function(Segmented_image_name);
    
    contours_totoal_number = size(detected_contours, 2);
    
    % generate edge pattern

    edgecontour_pattern = ones(max_row, max_col);
    for contour_single_number = 1 : contours_totoal_number 
        data = double(py.array.array('d',py.numpy.nditer(detected_contours{contour_single_number}))); % d is for double, see link below on types
        data = reshape(data,[2 detected_contours{contour_single_number}.size/2])'; %Could incorporate x.shape here ...
    
        for ii = 1 : size(data, 1)
            if (data(ii, 2) < max_row && data(ii, 2) > 0 && data(ii, 1) < max_col && data(ii, 1) > 0)
                edgecontour_pattern(data(ii, 2), data(ii, 1)) = 0;
            end
        end
    end

end