# -*- coding: utf-8 -*-
"""
Created on Sun Sep 29 22:51:55 2019

@author: User
"""

# -*- coding: utf-8 -*-
"""
Created on Thu Jun 20 21:05:28 2019

@author: LU Bo
"""

import cv2
import numpy as np 
def contour_function(image_name):
    #im = cv2.imread('segmented_suture_l.jpg')
    im =cv2.imread(image_name, 0)
    #imgray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(im, 127, 255, 0)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    
    selected_contours = [];
    for x in contours:
        if (np.size(x) > 400):
            selected_contours.append(x)
    
    return selected_contours
'''    
    bench_num = 0
    desired_one = []
    account = -1
    
    for single_list in contours:
        account  = account + 1
        if (single_list.shape[0] > bench_num):
            bench_num = single_list.shape[0]
            desired_one = account
        
    selected_contours = contours[desired_one]
    return selected_contours
'''  

