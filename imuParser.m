%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Function to take comma seperated imu input from an output file from
%%%% mti sdk and stores it in variables passed into the function
%%%% Created: Chris Squire
%%%% Date: 08/04/17
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [counter, time, accel, velInc, orienQuat, angleVel, absPos, vel] = imuParser(fileName)
% takes in the file name of IMU logger data and converts into matrix
% format/sensible data
% fileName: name & path of file
% counter: Array specifying which sample this is (in case it skips samples
% time: Array of timestamps in seconds (converted)
% accel: 3d matrix specifying accelerations in [x,y,z] for time t
% velInc: 3d matrix specifying velocity increment changes in [x,y,z] for time t
% orienQuat: Quaternion orientation for time t
% angleVel: [roll, pitch, yaw] for time tfunction imuParser
% absPos: [lat, lon, alt] for time t
% vel: [x,y,z] velocity for time t


    %Open file
    fid = fopen(fileName);
    if (fid == -1)
        str = sprintf("Couldn't open file %s", fileName);
        disp(str);
        return;
    end

    %otherwise successfully opened file
    %skip to start of logging data
    for i=1:6
        tline = fgets(fid);
    end

    %currently splitting using ; delimited opition
    i = 1;
    
    counter = zeros(1,1);
    time = zeros(1,1);
    accel = zeros(1,3);
    velInc = zeros(1,3);
    orienQuat = zeros(1,4);
    angleVel = zeros(1,3);
    absPos = zeros(1,3);
    vel = zeros(1,3);
    while ischar(tline)
        j = 1;
        tline = fgets(fid);
        if(~ischar(tline))
            break;
        end
        tmp = split(tline, ';'); %change this based on 
        data = zeros(size(tmp,1),size(tmp,2));
        data = str2double(tmp);
        counter(i, 1) = str2double(tmp(j));
        j = j+1;
        time(i, 1) = str2double(tmp(j))*1e-4;
        j = j+1;
        accel(i, :) = str2double(tmp(j:j+2));
        j = j + 3;
        velInc(i, :) = str2double(tmp(j:j+2));
        j = j+3;
        orienQuat(i, :) = str2double(tmp(j:j+3));
        j = j+4;
        angleVel(i,:) = str2double(tmp(j:j+2));
        j = j+3;
        absPos(i,:) = str2double(tmp(j:j+2));
        j = j+3;
        vel(i,:) = str2double(tmp(j:j+2));
        i = i+1;
    end
end
    



