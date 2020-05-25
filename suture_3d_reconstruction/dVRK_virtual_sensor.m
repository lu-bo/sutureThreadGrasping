classdef dVRK_virtual_sensor
    
    methods (Static)
        function transformation_matrix = get_transformation_matrix_object1_to_object2(object_1, object_2)
            vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
            vrep.simxFinish(-1); % just in case, close all opened connections
            clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
            vrep.simxSynchronous(clientID, true);
            
            [~,object1_handle] = vrep.simxGetObjectHandle(clientID, object_1, vrep.simx_opmode_blocking);
            [~,object2_handle] = vrep.simxGetObjectHandle(clientID, object_2, vrep.simx_opmode_blocking);

            [~, orientation] = vrep.simxGetObjectOrientation(clientID, object1_handle, object2_handle, vrep.simx_opmode_blocking);
            %[~, EulerAngles_2] = vrep.simxGetObjectOrientation(clientID, double(PSM_JOINT_ID(1,3)), double(PSM_JOINT_ID(1,1)), vrep.simx_opmode_blocking);
            [~, translation] = vrep.simxGetObjectPosition(clientID, object1_handle, object2_handle, vrep.simx_opmode_blocking);
            
            R_X = [1 0 0; 0 cos(orientation(1)) -sin(orientation(1)); 0 sin(orientation(1)) cos(orientation(1))];
            R_Y = [cos(orientation(2)) 0 sin(orientation(2)); 0 1 0; -sin(orientation(2)) 0 cos(orientation(2))];
            R_Z = [cos(orientation(3)) -sin(orientation(3)) 0; sin(orientation(3)) cos(orientation(3)) 0; 0 0 1];
            
            Rotation_matrix = R_X * R_Y * R_Z;
            transformation_matrix = [Rotation_matrix, translation'; 0 0 0 1];
            %vrep.delete(); % call the destructor!
            vrep.simxSynchronousTrigger(clientID);
        end
        
        function [image_left, image_right] = get_stereo_vision()
            vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
            vrep.simxFinish(-1); % just in case, close all opened connections
            clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
            vrep.simxSynchronous(clientID, true);
            
            [~, left_visionSensor]  = vrep.simxGetObjectHandle(clientID, 'Vision_sensor_left', vrep.simx_opmode_blocking);
            [~, right_visionSensor] = vrep.simxGetObjectHandle(clientID, 'Vision_sensor_right', vrep.simx_opmode_blocking);

            [~, ~,  image_left] = vrep.simxGetVisionSensorImage2(clientID, left_visionSensor,  0, vrep.simx_opmode_blocking); % '0' indicates -> show RGB image.
            [~, ~, image_right] = vrep.simxGetVisionSensorImage2(clientID, right_visionSensor, 0, vrep.simx_opmode_blocking);
            %image_left = imrotate(image_left, 180); image_right = imrotate(image_right, 180);

            %subplot(1, 2, 1); imshow(image_left); subplot(1, 2 ,2); imshow(image_right);
            %vrep.delete(); % call the destructor!
            %imwrite(image_left,'left_image_6.jpg'); imwrite(image_right,'right_image_6.jpg')
            vrep.simxSynchronousTrigger(clientID);
        end
        
        function show_stereo_images(left_image, right_image)
                subplot(1, 2, 1); imshow(left_image); subplot(1, 2 ,2); imshow(right_image);
        end
        
    end
end