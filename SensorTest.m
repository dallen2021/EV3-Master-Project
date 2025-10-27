% sensorTest.m - continuously read color, touch, and distance sensors for debugging
% authors: Daniel Allen, Nathan Alarcon, Thomas Stott, Ian Gallegos

% sensor ports
colorPort = 2;      % floor detection
touchPort = 3;      % front wall detection
distancePort = 4;   % side wall detection

% motor ports
rightMotor = 'A';
leftMotor = 'C';

% check brick is connected
if ~exist('brick', 'var')
    error('Brick not connected! Please run connectEV3.m first.');
end

% set color sensor to reflected light mode
try
    brick.SetColorMode(colorPort, 0);
    disp('Color sensor mode set successfully.');
catch ME
    disp('ERROR: Could not set color sensor mode.');
    error('Connection error: %s', ME.message);
end

global key
InitKeyboard();

disp('========================================');
disp('EV3 Maze Navigation Sensor Testing');
disp('========================================');
disp('Color Sensor (Port 2): Floor brightness');
disp('Touch Sensor (Port C): Front wall collision');
disp('Distance Sensor (Port 4): Side wall distance');
disp(' ');
disp('Press Q to quit');
disp('========================================');

% continuously read sensors
while key ~= 'q'
    pause(0.5);

    % read sensors
    brightness = brick.LightReflect(colorPort);
    touchPressed = brick.TouchPressed(touchPort);
    distance = brick.UltrasonicDist(distancePort);

    % categorize floor brightness
    if brightness < 20
        floorColor = 'BLACK (dark)';
    elseif brightness > 50
        floorColor = 'WHITE (light)';
    else
        floorColor = 'GRAY (medium)';
    end

    % categorize front wall status
    if touchPressed == 1
        frontWall = 'WALL HIT!';
        wallStatus = '***';
    else
        frontWall = 'Clear';
        wallStatus = '   ';
    end

    % categorize side wall distance
    if distance < 10
        sideWall = 'WALL VERY CLOSE!';
    elseif distance < 20
        sideWall = 'Wall nearby';
    elseif distance < 50
        sideWall = 'Wall detected';
    else
        sideWall = 'Open space';
    end

    % display sensor readings
    disp('========================================');
    disp(sprintf('Floor: %d - %s', brightness, floorColor));
    disp(sprintf('Front: %s %s', frontWall, wallStatus));
    disp(sprintf('Side: %.1f cm - %s', distance, sideWall));

    % beep if front wall hit
    if touchPressed == 1
        brick.beep();
        disp('>>> FRONT COLLISION DETECTED! <<<');
    end

    % warn if side wall very close
    if distance < 10
        disp('>>> SIDE WALL VERY CLOSE! <<<');
    end

end

CloseKeyboard();
disp('Sensor test ended.');
