function optimized_nodes = dVRK_suture_thread_smoothing(input_nodes)
    %Savitzky¨CGolay filter (sgolayfilt) - smoothing individual axes
    windowWidth = 5; %Standard example values
    polynomialOrder = 3;

    xsg = sgolayfilt(input_nodes(:,1), polynomialOrder, windowWidth);
    ysg = sgolayfilt(input_nodes(:,2), polynomialOrder, windowWidth);
    zsg = sgolayfilt(input_nodes(:,3), polynomialOrder, windowWidth);
    optimized_nodes = [xsg, ysg, zsg];
end