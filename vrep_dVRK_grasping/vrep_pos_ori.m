function [Position, EulerAngles] = vrep_pos_ori(cur_ob, relative_obj)

    vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1); % just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
    vrep.simxSynchronous(clientID, true);

    [~, Current_system_handle] = vrep.simxGetObjectHandle(clientID, cur_ob, vrep.simx_opmode_blocking);
    [~, relative_system_handle]  = vrep.simxGetObjectHandle(clientID, relative_obj, vrep.simx_opmode_blocking);
    [~, EulerAngles] = vrep.simxGetObjectOrientation(clientID, Current_system_handle, relative_system_handle, vrep.simx_opmode_blocking);
    [~, Position]    = vrep.simxGetObjectPosition(clientID, Current_system_handle, relative_system_handle, vrep.simx_opmode_blocking);

    vrep.simxSynchronousTrigger(clientID);
end