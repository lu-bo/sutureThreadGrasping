function [suture_position ] = setup_initialization()

    vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1); % just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
    vrep.simxSynchronous(clientID, true);
    
    % ---- extract reference coordinate and setup suture
    [~, origin_ID] = vrep.simxGetObjectHandle(clientID, 'ResizableFloor_5_25',  vrep.simx_opmode_blocking);
    [~, suture_ID] = vrep.simxGetObjectHandle(clientID, 'suture_150mm_horizon', vrep.simx_opmode_blocking);

    initial_suture_position = [-1.5675   -0.005    0.70];
    initial_suture_orien    = [90, 0, 0] * pi / 180;

    [~, suture_position] = vrep.simxGetObjectPosition(clientID, suture_ID, origin_ID, vrep.simx_opmode_blocking);
    [~, suture_orien] = vrep.simxGetObjectOrientation(clientID, suture_ID, origin_ID, vrep.simx_opmode_blocking);

    vrep.simxSetObjectOrientation(clientID, suture_ID, origin_ID, initial_suture_orien , vrep.simx_opmode_blocking);
    vrep.simxSetObjectPosition(clientID, suture_ID, origin_ID, initial_suture_position, vrep.simx_opmode_blocking);

    % ---- set phantom's position
    [~, phantom_ID] = vrep.simxGetObjectHandle(clientID, 'Phantom_respondable', vrep.simx_opmode_blocking);
    [~, phantom_position] = vrep.simxGetObjectPosition(clientID, phantom_ID, origin_ID, vrep.simx_opmode_blocking);
    Initial_phantom_position = [-1.56072    0    0.58];
    vrep.simxSetObjectPosition(clientID, phantom_ID, origin_ID, Initial_phantom_position, vrep.simx_opmode_blocking);

    % ---- extract ECM information and setup initial config
    [~, J1_ECM] = vrep.simxGetObjectHandle(clientID, 'J1_ECM', vrep.simx_opmode_blocking);
    [~, J2_ECM] = vrep.simxGetObjectHandle(clientID, 'J2_ECM', vrep.simx_opmode_blocking);
    [~, J3_ECM] = vrep.simxGetObjectHandle(clientID, 'J3_ECM', vrep.simx_opmode_blocking);
    [~, J4_ECM] = vrep.simxGetObjectHandle(clientID, 'J4_ECM', vrep.simx_opmode_blocking);

    [~, J1_ECM_init_position] = vrep.simxGetJointPosition(clientID, J1_ECM, vrep.simx_opmode_blocking);
    [~, J2_ECM_init_position] = vrep.simxGetJointPosition(clientID, J2_ECM, vrep.simx_opmode_blocking);
    [~, J3_ECM_init_position] = vrep.simxGetJointPosition(clientID, J3_ECM, vrep.simx_opmode_blocking);
    [~, J4_ECM_init_position] = vrep.simxGetJointPosition(clientID, J4_ECM, vrep.simx_opmode_blocking);

    %vrep.simxSetJointPosition(clientID, J1_ECM, 0, vrep.simx_opmode_blocking);
    vrep.simxSetJointPosition(clientID, J2_ECM, -10*pi/180, vrep.simx_opmode_blocking);  
    
    % ---- set suture's position and orientation
    [~, suture_ID] = vrep.simxGetObjectHandle(clientID, 'suture_15mm_horizon', vrep.simx_opmode_blocking);
    [~, suture_position] = vrep.simxGetObjectPosition(clientID, suture_ID, origin_ID, vrep.simx_opmode_blocking);
    
    vrep.simxSynchronousTrigger(clientID);
end