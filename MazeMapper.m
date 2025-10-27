% mazeMapper.m - zone-based right-wall following maze navigation
% hardware: color (port 2), touch (port C/3), distance (port 4), motors (A, C)
% press 'q' to stop
% authors: Daniel Allen, Nathan Alarcon, Thomas Stott, Ian Gallegos

% sensor ports
colorPort = 2;
touchPort = 3;
distancePort = 4;

% motor ports
rightMotor = 'A';
leftMotor = 'C';

% navigation parameters
SPEED = 30;
TURN_SPEED = 50;
STEER_SPEED = 25;
BLACK_THRESHOLD = 20;

% zone-based wall following parameters
TARGET_WALL_DISTANCE = 20;
TOLERANCE = 6;
MIN_DISTANCE = TARGET_WALL_DISTANCE - TOLERANCE;
MAX_DISTANCE = TARGET_WALL_DISTANCE + TOLERANCE;
NO_WALL_THRESHOLD = 50;

% check brick is connected
if ~exist('brick', 'var')
    error('Brick not connected! Please run connectEV3.m first.');
end

% set color sensor mode
try
    brick.SetColorMode(colorPort, 0);
    disp('Color sensor initialized.');
catch ME
    error('Could not initialize sensors. Check connection and run connectEV3 again.');
end

% initialize keyboard for exit
global key
InitKeyboard();

disp('========================================');
disp('AUTONOMOUS MAZE MAPPER - ZONE BASED');
disp('========================================');
disp('The robot will now navigate automatically');
disp('Strategy: Right-wall following with zones');
disp(sprintf('Target distance: %d cm (tolerance ±%d cm)', TARGET_WALL_DISTANCE, TOLERANCE));
disp(sprintf('  GOOD ZONE: %.0f-%.0f cm = go straight', MIN_DISTANCE, MAX_DISTANCE));
disp(sprintf('  TOO CLOSE: < %.0f cm = steer left', MIN_DISTANCE));
disp(sprintf('  TOO FAR: > %.0f cm = steer right', MAX_DISTANCE));
disp(' ');
disp('Sensors:');
disp('  - Touch (Port C): Front wall detection');
disp('  - Distance (Port 4): RIGHT wall distance');
disp('  - Color (Port 2): Floor boundary detection');
disp(' ');
disp('Press Q to stop');
disp('========================================');
pause(2);

% main navigation loop
disp('Starting autonomous navigation...');

% track current state to avoid redundant motor commands
currentState = 'none';

while key ~= 'q'
    % read all sensors
    brightness = brick.LightReflect(colorPort);
    touchPressed = brick.TouchPressed(touchPort);
    rightWallDistance = brick.UltrasonicDist(distancePort);

    % display current status
    disp(sprintf('Floor: %d | Touch: %d | Right Wall: %.1f cm', brightness, touchPressed, rightWallDistance));

    % priority 1: check for black boundary (out of arena)
    if brightness < BLACK_THRESHOLD
        disp('>>> BLACK BOUNDARY! Backing up and turning...');

        % Stop
        brick.StopMotor([rightMotor leftMotor], 'Brake');
        pause(0.3);

        % Back up
        brick.MoveMotor(rightMotor, -SPEED);
        brick.MoveMotor(leftMotor, -SPEED);
        pause(1);

        % Turn 90 degrees LEFT
        brick.MoveMotor(rightMotor, TURN_SPEED);
        brick.MoveMotor(leftMotor, -TURN_SPEED);
        pause(0.8);

        % Stop
        brick.StopMotor([rightMotor leftMotor], 'Brake');
        pause(0.3);

    % priority 2: front wall collision detected
    elseif touchPressed == 1
        disp('>>> FRONT WALL HIT! Backing up and turning LEFT...');
        brick.beep();

        % Stop
        brick.StopMotor([rightMotor leftMotor], 'Brake');
        pause(0.3);

        % Back up
        brick.MoveMotor(rightMotor, -SPEED);
        brick.MoveMotor(leftMotor, -SPEED);
        pause(1);

        % Turn 90 degrees LEFT (away from wall)
        brick.MoveMotor(rightMotor, TURN_SPEED);
        brick.MoveMotor(leftMotor, -TURN_SPEED);
        pause(3);

        % Stop briefly then let wall-following resume
        brick.StopMotor([rightMotor leftMotor], 'Brake');
        pause(0.2);

    % zone 1: no wall detected - turn right to find wall
    elseif rightWallDistance > NO_WALL_THRESHOLD
        if ~strcmp(currentState, 'finding_wall')
            disp(sprintf('>>> NO WALL (%.1f cm) - turning right to find wall', rightWallDistance));
            brick.MoveMotor(rightMotor, SPEED * 0.5);
            brick.MoveMotor(leftMotor, SPEED);
            currentState = 'finding_wall';
        end

    % zone 2: too close - steer left away from wall
    elseif rightWallDistance < MIN_DISTANCE
        if ~strcmp(currentState, 'steering_left')
            disp(sprintf('  << TOO CLOSE (%.1f cm) - STEERING LEFT', rightWallDistance));
            brick.MoveMotor(rightMotor, SPEED);
            brick.MoveMotor(leftMotor, STEER_SPEED);
            currentState = 'steering_left';
        end

    % zone 3: too far - steer right toward wall
    elseif rightWallDistance > MAX_DISTANCE
        if ~strcmp(currentState, 'steering_right')
            disp(sprintf('  >> TOO FAR (%.1f cm) - STEERING RIGHT', rightWallDistance));
            brick.MoveMotor(rightMotor, STEER_SPEED);
            brick.MoveMotor(leftMotor, SPEED);
            currentState = 'steering_right';
        end

    % zone 4: good zone - go straight
    else
        if ~strcmp(currentState, 'straight')
            disp(sprintf('  == GOOD ZONE (%.1f cm) - STRAIGHT', rightWallDistance));
            brick.MoveMotor(rightMotor, SPEED);
            brick.MoveMotor(leftMotor, SPEED);
            currentState = 'straight';
        end
    end

    % small delay to let motors respond before next sensor reading
    pause(0.05);
end

% cleanup - stop motors and close keyboard
brick.StopMotor([rightMotor leftMotor], 'Coast');
CloseKeyboard();
disp('========================================');
disp('Maze mapping stopped.');
disp('========================================');
