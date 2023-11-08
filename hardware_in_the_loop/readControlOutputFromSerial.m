function actuatorData = readControlOutputFromSerial(board)

%{
-----------DESCRIPTION OF FUNCTION:------------------
Use the serial communication to read control output from the microcontroller running the obsw

INPUT:
    board:          string defining what board is connected (either "main" or "payload")

OUTPUT:
    actuatorData:   struct containing all data received from obsw

%}

% Author: Emilio Corigliano
% Skyward Experimental Rocketry | ELC Dept | electronics@skywarder.eu
% email: emilio.corigliano@skywarder.eu
% Release date: 10/03/2021

% Author: Pier Francesco Bachini
% Skyward Experimental Rocketry | AVN Dept
% email: pierfrancesco.bachini@skywarder.eu
% Revision date: 27/08/2023

switch board
    case "main"
        % Receive data from serial comunication
        obswVals = serialbridge("Read", 31);

        actuatorData.ada.mslAltitude = obswVals(1);
        actuatorData.ada.aglAltitude = obswVals(2);
        actuatorData.ada.verticalSpeed = obswVals(3);
        actuatorData.ada.apogeeDetected = obswVals(4);
        actuatorData.ada.updating = obswVals(5);
        actuatorData.nas.n = obswVals(6);
        actuatorData.nas.e = obswVals(7);
        actuatorData.nas.d = obswVals(8);
        actuatorData.nas.vn = obswVals(9);
        actuatorData.nas.ve = obswVals(10);
        actuatorData.nas.vd = obswVals(11);
        actuatorData.nas.qx = obswVals(12);
        actuatorData.nas.qy = obswVals(13);
        actuatorData.nas.qz = obswVals(14);
        actuatorData.nas.qw = obswVals(15);
        actuatorData.nas.updating = obswVals(16);
        actuatorData.abk.updating = obswVals(17);
        actuatorData.mea.correctedPressure = obswVals(18);
        actuatorData.mea.estimatedMass = obswVals(19);
        actuatorData.mea.estimatedApogee = obswVals(20);
        actuatorData.mea.updating = obswVals(21);
        actuatorData.actuators.airbrakesPercentage = obswVals(22);
        actuatorData.actuators.expulsionPercentage = obswVals(23);
        actuatorData.actuators.mainValvePercentage = obswVals(24);
        actuatorData.actuators.ventingValvePercentage = obswVals(25);
        actuatorData.flags.flag_flight = logical(obswVals(26));
        actuatorData.flags.flag_ascent = logical(obswVals(27));
        actuatorData.flags.flag_burning = logical(obswVals(28));
        actuatorData.flags.flag_airbrakes = logical(obswVals(29));
        actuatorData.flags.flag_para1 = logical(obswVals(30));
        actuatorData.flags.flag_para2 = logical(obswVals(31));

        disp(actuatorData.actuators);
    case "payload"
        % Receive data from serial comunication
        obswVals = serialbridge("Read", 30);

        actuatorData.nas.n = obswVals(1);
        actuatorData.nas.e = obswVals(2);
        actuatorData.nas.d = obswVals(3);
        actuatorData.nas.vn = obswVals(4);
        actuatorData.nas.ve = obswVals(5);
        actuatorData.nas.vd = obswVals(6);
        actuatorData.nas.qx = obswVals(7);
        actuatorData.nas.qy = obswVals(8);
        actuatorData.nas.qz = obswVals(9);
        actuatorData.nas.qw = obswVals(10);
        actuatorData.nas.updating = obswVals(11);
        actuatorData.actuators.airbrakesPercentage = obswVals(12);
        actuatorData.actuators.expulsionPercentage = obswVals(13);
        actuatorData.actuators.parafoilLeftPercentage = obswVals(14);
        actuatorData.actuators.parafoilRightPercentage = obswVals(15);
        actuatorData.actuators.mainValvePercentage = obswVals(16);
        actuatorData.actuators.ventingValvePercentage = obswVals(17);
        actuatorData.actuators.releaseValvePercentage = obswVals(18);
        actuatorData.actuators.fillingValvePercentage = obswVals(19);
        actuatorData.actuators.disconnectValvePercentage = obswVals(20);
        actuatorData.wes.windX = obswVals(21);
        actuatorData.wes.windY = obswVals(22);
        actuatorData.guidance.psiRef = obswVals(23);
        actuatorData.guidance.deltaA = obswVals(24);
        actuatorData.flags.flag_flight = logical(obswVals(25));
        actuatorData.flags.flag_ascent = logical(obswVals(26));
        actuatorData.flags.flag_burning = logical(obswVals(27));
        actuatorData.flags.flag_airbrakes = logical(obswVals(28));
        actuatorData.flags.flag_para1 = logical(obswVals(29));
        actuatorData.flags.flag_para2 = logical(obswVals(30));
end
