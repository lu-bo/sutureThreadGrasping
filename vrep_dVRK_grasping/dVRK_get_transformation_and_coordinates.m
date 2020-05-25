function [RotationMatrix, ...
          Position, ...
          Transformed_coordinates] = ...
          dVRK_get_transformation_and_coordinates(Current_system, Target_system, Coordinates_in_current_system)
    % the returned rotation and translation matrix should be applied to the
    % data with unit of 'meter'!!! 

    vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1); % just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
    vrep.simxSynchronous(clientID, true);
    
    %[~, Current_system_handle] = vrep.simxGetObjectHandle(clientID, 'Vision_sensor_left', vrep.simx_opmode_blocking);
    %[~, Target_system_handle]  = vrep.simxGetObjectHandle(clientID, 'J1_ECM', vrep.simx_opmode_blocking);
    
    [~, Current_system_handle] = vrep.simxGetObjectHandle(clientID, Current_system, vrep.simx_opmode_blocking);
    [~, Target_system_handle]  = vrep.simxGetObjectHandle(clientID, Target_system, vrep.simx_opmode_blocking);

    
    %[~, matrix]=vrep.simxGetJointMatrix(clientID, RCM_vision_system, vrep.simx_opmode_blocking)
    
    [~, EulerAngles] = vrep.simxGetObjectOrientation(clientID, Current_system_handle, Target_system_handle, vrep.simx_opmode_blocking);
    [~, Position]    = vrep.simxGetObjectPosition(clientID, Current_system_handle, Target_system_handle, vrep.simx_opmode_blocking);
    
    R_X = [1 0 0; 0 cos(EulerAngles(1)) -sin(EulerAngles(1)); 0 sin(EulerAngles(1)) cos(EulerAngles(1))];
    R_Y = [cos(EulerAngles(2)) 0 sin(EulerAngles(2)); 0 1 0; -sin(EulerAngles(2)) 0 cos(EulerAngles(2))];
    R_Z = [cos(EulerAngles(3)) -sin(EulerAngles(3)) 0; sin(EulerAngles(3)) cos(EulerAngles(3)) 0; 0 0 1];
    
    RotationMatrix = R_X*R_Y*R_Z;
    
    % --------------- 3D coordinates transformation -----------------------
    % The unit of the coordinates should be 'm', be cautious!
            
    Transformation_Matrix = [RotationMatrix ...
                             Position'; ...
                             0 0 0 1];
      
    Transformed_coordinates = Transformation_Matrix * ...
                              [Coordinates_in_current_system'; ...
                              repmat(1, 1, size(Coordinates_in_current_system, 1))];
                          
    Transformed_coordinates = Transformed_coordinates(1 : 3, :)';
    
    vrep.simxSynchronousTrigger(clientID);
    %vrep.delete(); % call the destructor!
end