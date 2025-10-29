% mazeMapper_Gyro.m - gyro-assisted right-wall following with precise turns
% hardware: distance (port 1), gyro (port 2, backwards), touch (port C/3), color (port 4), motors (A, C, D)
% press 'q' to stop
% authors: Daniel Allen, Nathan Alarcon, Thomas Stott, Ian Gallegos

% sensor ports
distancePort = 1;   % ultrasonic distance sensor (right wall)
gyroPort = 2;       % gyro sensor (mounted backwards)
touchPort = 3;      % touch sensor (front wall)
colorPort = 4;      % color sensor (floor detection)

% motor ports
rightMotor = 'A';
leftMotor = 'C';
flipperMotor = 'D';  % medium motor for flipper

% navigation parameters
SPEED = 50;
TURN_SPEED = 30;
TARGET_WALL_DISTANCE = 25;
WALL_TOLERANCE = 5;
NO_WALL_THRESHOLD = 50;

% gyro parameters
HEADING_TOLERANCE = 1;  % reduced from 5 for tighter control
TURN_ANGLE = 90;
ANGLE_OVERSHOOT = 2;

% color detection parameters
COLOR_RED = 5;      % EV3 color code for red
COLOR_BLUE = 2;     % EV3 color code for blue
COLOR_GREEN = 3;    % EV3 color code for green

% check brick is connected
if ~exist('brick', 'var')
    error('Brick not connected! Please run connectEV3.m first.');
end

% initialize color sensor
try
    brick.SetColorMode(colorPort, 2);  % mode 2 = color detection
    disp('Color sensor initialized.');
catch ME
    error('Could not initialize color sensor. Check connection.');
end

% initialize gyro sensor
try
    brick.GyroCalibrate(gyroPort);
    pause(1);
    disp('Gyro sensor calibrated.');
catch ME
    error('Could not initialize gyro sensor. Check connection.');
end

% initialize flipper motor - rotate 45 degrees counterclockwise
try
    disp('Positioning flipper motor...');
    brick.MoveMotorAngleRel(flipperMotor, 50, -45, 'Brake');
    pause(0.5);
    disp('Flipper motor positioned at -45 degrees.');
catch ME
    error('Could not initialize flipper motor. Check connection.');
end

% initialize keyboard for exit
global key
InitKeyboard();

disp('========================================');
disp('MAZE MAPPER - GYRO-ASSISTED');
disp('========================================');
disp('Gyro-based precise navigation');
disp(sprintf('Target wall distance: %d cm (±%d cm)', TARGET_WALL_DISTANCE, WALL_TOLERANCE));
disp(sprintf('Heading tolerance: ±%d degrees', HEADING_TOLERANCE));
disp(' ');
disp('Gyro benefits:');
disp('  - Accurate 90-degree turns');
disp('  - Straight-line correction');
disp('  - No motor/battery drift');
disp('  - Heading awareness');
disp(' ');
disp('Press Q to stop');
disp('========================================');
pause(2);

% reset gyro angle to 0 at start
brick.GyroCalibrate(gyroPort);
pause(0.5);

% track target heading (changes by 90° with each turn)
targetHeading = 0;

disp('Starting autonomous navigation...');

while key ~= 'q'
    pause(0.05);

    % read all sensors
    currentHeading = -brick.GyroAngle(gyroPort);  % negate because gyro is mounted backwards
    touchPressed = brick.TouchPressed(touchPort);
    rightWallDistance = brick.UltrasonicDist(distancePort);
    detectedColor = brick.ColorCode(colorPort);  % read color

    % calculate heading error
    headingError = currentHeading - targetHeading;

    % normalize heading error to -180 to 180 range
    while headingError > 180
        headingError = headingError - 360;
    end
    while headingError < -180
        headingError = headingError + 360;
    end

    % display status
    disp(sprintf('Heading: %.1f° | Target: %.1f° | Error: %.1f° | Wall: %.1f cm | Touch: %d | Color: %d', ...
                 currentHeading, targetHeading, headingError, rightWallDistance, touchPressed, detectedColor));

    % priority 1: color detection - red stops and goes, blue/green enter manual control
    if detectedColor == COLOR_RED
        disp('>>> RED DETECTED! Stopping for 2 seconds then continuing...');
        brick.StopMotor([rightMotor leftMotor], 'Brake');
        pause(2);

        % move forward to clear the color with gyro control
        disp('    Moving forward to clear the color...');
        forwardStartTime = tic;
        while toc(forwardStartTime) < 0.5  % 0.5 seconds forward
            currentHeading = -brick.GyroAngle(gyroPort);
            headingError = currentHeading - targetHeading;

            % normalize error
            while headingError > 180
                headingError = headingError - 360;
            end
            while headingError < -180
                headingError = headingError + 360;
            end

            % correct heading while moving forward
            if headingError > 2
                brick.MoveMotor(rightMotor, SPEED * 0.7);
                brick.MoveMotor(leftMotor, SPEED);
            elseif headingError < -2
                brick.MoveMotor(rightMotor, SPEED);
                brick.MoveMotor(leftMotor, SPEED * 0.7);
            else
                brick.MoveMotor(rightMotor, SPEED);
                brick.MoveMotor(leftMotor, SPEED);
            end
            pause(0.02);
        end
        brick.StopMotor([rightMotor leftMotor], 'Brake');
        pause(0.2);
        disp('    Resuming navigation...');

    elseif detectedColor == COLOR_BLUE || detectedColor == COLOR_GREEN
        disp('>>> COLOR DETECTED! Entering manual control mode...');
        disp('    Controls: W=forward, S=backward, A=turn left, D=turn right');
        disp('    Up Arrow=flipper up, Down Arrow=flipper down');
        disp('    Press R to resume autonomous navigation');
        brick.StopMotor([rightMotor leftMotor], 'Brake');

        % manual control loop
        while key ~= 'q'
            pause(0.05);

            % check for resume autonomous
            if key == 'r'
                disp('>>> Resuming autonomous navigation...');
                brick.StopMotor([rightMotor leftMotor], 'Brake');

                % reinitialize gyro angle
                disp('    Reinitializing gyro...');
                brick.GyroCalibrate(gyroPort);
                pause(0.5);
                targetHeading = 0;  % reset target heading
                disp('    Gyro reinitialized. Resuming...');
                break;

            % movement controls
            elseif key == 'w'
                % forward
                brick.MoveMotor(rightMotor, SPEED);
                brick.MoveMotor(leftMotor, SPEED);
            elseif key == 's'
                % backward
                brick.MoveMotor(rightMotor, -SPEED);
                brick.MoveMotor(leftMotor, -SPEED);
            elseif key == 'a'
                % turn left (right motor forward, left motor backward)
                brick.MoveMotor(rightMotor, TURN_SPEED);
                brick.MoveMotor(leftMotor, -TURN_SPEED);
            elseif key == 'd'
                % turn right (left motor forward, right motor backward)
                brick.MoveMotor(rightMotor, -TURN_SPEED);
                brick.MoveMotor(leftMotor, TURN_SPEED);

            % flipper controls (arrow keys return numeric codes)
            elseif (ischar(key) && strcmp(key, 'uparrow')) || (isnumeric(key) && key == 30)
                % flipper up (counterclockwise) - continuous movement
                brick.MoveMotor(flipperMotor, -30);
            elseif (ischar(key) && strcmp(key, 'downarrow')) || (isnumeric(key) && key == 31)
                % flipper down (clockwise) - continuous movement
                brick.MoveMotor(flipperMotor, 30);

            % no key pressed - stop all motors
            elseif key == 0
                brick.StopMotor([rightMotor leftMotor flipperMotor], 'Brake');
            end
        end

    % priority 2: front wall collision - make accurate 90° left turn
    elseif touchPressed == 1
        disp('>>> FRONT WALL! Executing precise 90° LEFT turn with gyro...');
        brick.beep();

        % Stop
        brick.StopMotor([rightMotor leftMotor], 'Brake');
        pause(0.3);

        % Back up slightly (gyro-controlled to stay straight)
        backupStartTime = tic;
        while toc(backupStartTime) < 1.2
            currentHeading = -brick.GyroAngle(gyroPort);
            headingError = currentHeading - targetHeading;

            % normalize error
            while headingError > 180
                headingError = headingError - 360;
            end
            while headingError < -180
                headingError = headingError + 360;
            end

            % correct heading while backing up (reversed correction)
            if headingError > 2
                brick.MoveMotor(rightMotor, -SPEED);
                brick.MoveMotor(leftMotor, -SPEED * 0.7);
            elseif headingError < -2
                brick.MoveMotor(rightMotor, -SPEED * 0.7);
                brick.MoveMotor(leftMotor, -SPEED);
            else
                brick.MoveMotor(rightMotor, -SPEED);
                brick.MoveMotor(leftMotor, -SPEED);
            end
            pause(0.02);
        end

        % Stop
        brick.StopMotor([rightMotor leftMotor], 'Brake');
        pause(0.2);

        % Update target heading (turn left = +90 degrees)
        targetHeading = targetHeading + TURN_ANGLE;

        % Execute gyro-controlled turn
        disp(sprintf('    Turning from %.1f° to %.1f°...', currentHeading, targetHeading));

        % Turn until we reach target angle (with overshoot compensation)
        turnTarget = targetHeading - ANGLE_OVERSHOOT;

        while key ~= 'q'
            currentHeading = -brick.GyroAngle(gyroPort);  % Negate because gyro is mounted backwards
            headingError = currentHeading - turnTarget;

            % Normalize error
            while headingError > 180
                headingError = headingError - 360;
            end
            while headingError < -180
                headingError = headingError + 360;
            end

            if headingError >= -2  % Close enough to target
                break;
            end

            % Turn left (right motor forward, left motor backward)
            brick.MoveMotor(rightMotor, TURN_SPEED);
            brick.MoveMotor(leftMotor, -TURN_SPEED);
            pause(0.02);
        end

        % Stop and settle
        brick.StopMotor([rightMotor leftMotor], 'Brake');
        pause(0.3);

        disp(sprintf('    Turn complete! New heading: %.1f°', -brick.GyroAngle(gyroPort)));

    % priority 3: no wall detected - make precise 90° right turn to go down pathway
    elseif rightWallDistance > NO_WALL_THRESHOLD
        disp(sprintf('>>> NO WALL DETECTED (%.1f cm) - Executing 90° RIGHT turn...', rightWallDistance));
        brick.beep();

        % Move forward briefly to clear the corner before turning (gyro-controlled)
        disp('    Moving forward to clear corner...');
        forwardStartTime = tic;
        while toc(forwardStartTime) < 1.1
            currentHeading = -brick.GyroAngle(gyroPort);
            headingError = currentHeading - targetHeading;
            detectedColor = brick.ColorCode(colorPort);  % check for colors during movement

            % stop if color detected
            if detectedColor == COLOR_RED || detectedColor == COLOR_BLUE || detectedColor == COLOR_GREEN
                disp('    Color detected during corner clearing - stopping early');
                break;
            end

            % normalize error
            while headingError > 180
                headingError = headingError - 360;
            end
            while headingError < -180
                headingError = headingError + 360;
            end

            % correct heading while moving forward
            if headingError > 2
                brick.MoveMotor(rightMotor, SPEED * 0.7);
                brick.MoveMotor(leftMotor, SPEED);
            elseif headingError < -2
                brick.MoveMotor(rightMotor, SPEED);
                brick.MoveMotor(leftMotor, SPEED * 0.7);
            else
                brick.MoveMotor(rightMotor, SPEED);
                brick.MoveMotor(leftMotor, SPEED);
            end
            pause(0.02);
        end

        % Stop
        brick.StopMotor([rightMotor leftMotor], 'Brake');
        pause(0.3);

        % Update target heading (turn right = -90 degrees)
        targetHeading = targetHeading - TURN_ANGLE;

        % Execute gyro-controlled RIGHT turn
        disp(sprintf('    Turning from %.1f° to %.1f°...', -brick.GyroAngle(gyroPort), targetHeading));

        % Turn until we reach target angle (with overshoot compensation)
        turnTarget = targetHeading + ANGLE_OVERSHOOT;  % Add overshoot for right turn

        while key ~= 'q'
            currentHeading = -brick.GyroAngle(gyroPort);  % Negate because gyro is mounted backwards
            headingError = currentHeading - turnTarget;

            % Normalize error
            while headingError > 180
                headingError = headingError - 360;
            end
            while headingError < -180
                headingError = headingError + 360;
            end

            if headingError <= 2  % Close enough to target
                break;
            end

            % Turn right (left motor forward, right motor backward)
            brick.MoveMotor(rightMotor, -TURN_SPEED);
            brick.MoveMotor(leftMotor, TURN_SPEED);
            pause(0.02);
        end

        % Stop and settle
        brick.StopMotor([rightMotor leftMotor], 'Brake');
        pause(0.3);

        disp(sprintf('    Turn complete! New heading: %.1f°', -brick.GyroAngle(gyroPort)));

        % Move forward briefly after turning to clear the opening (gyro-controlled)
        disp('    Moving forward to enter new pathway...');
        forwardStartTime = tic;
        while toc(forwardStartTime) < 3.5
            currentHeading = -brick.GyroAngle(gyroPort);
            headingError = currentHeading - targetHeading;
            detectedColor = brick.ColorCode(colorPort);  % check for colors during movement

            % stop if color detected
            if detectedColor == COLOR_RED || detectedColor == COLOR_BLUE || detectedColor == COLOR_GREEN
                disp('    Color detected during pathway entry - stopping early');
                break;
            end

            % normalize error
            while headingError > 180
                headingError = headingError - 360;
            end
            while headingError < -180
                headingError = headingError + 360;
            end

            % correct heading while moving forward
            if headingError > 2
                brick.MoveMotor(rightMotor, SPEED * 0.7);
                brick.MoveMotor(leftMotor, SPEED);
            elseif headingError < -2
                brick.MoveMotor(rightMotor, SPEED);
                brick.MoveMotor(leftMotor, SPEED * 0.7);
            else
                brick.MoveMotor(rightMotor, SPEED);
                brick.MoveMotor(leftMotor, SPEED);
            end
            pause(0.02);
        end

        % Stop briefly before resuming wall following
        brick.StopMotor([rightMotor leftMotor], 'Brake');
        pause(0.2);

    % priority 4: gyro-assisted wall following
    else
        % maintain correct heading, then adjust for wall distance

        % determine wall correction needed
        wallError = rightWallDistance - TARGET_WALL_DISTANCE;

        % use gyro to maintain straight heading, plus wall correction
        if abs(headingError) > HEADING_TOLERANCE
            % heading drift detected - correct it aggressively
            if headingError > 0
                % drifting left (positive angle) - turn right (slow down right motor)
                disp(sprintf('  GYRO CORRECT >> (heading %.1f°) - slowing RIGHT motor', headingError));
                brick.MoveMotor(rightMotor, SPEED * 0.7);  % more aggressive correction
                brick.MoveMotor(leftMotor, SPEED);
            else
                % drifting right (negative angle) - turn left (slow down left motor)
                disp(sprintf('  GYRO CORRECT << (heading %.1f°) - slowing LEFT motor', headingError));
                brick.MoveMotor(rightMotor, SPEED);
                brick.MoveMotor(leftMotor, SPEED * 0.7);  % more aggressive correction
            end

        elseif abs(wallError) > WALL_TOLERANCE
            % heading good, but wall distance needs correction
            if wallError < 0
                % too close to wall - steer left
                disp(sprintf('  WALL ADJUST << (%.1f cm)', rightWallDistance));
                brick.MoveMotor(rightMotor, SPEED);
                brick.MoveMotor(leftMotor, SPEED * 0.75);  % gentler wall adjustment
            else
                % too far from wall - steer right
                disp(sprintf('  WALL ADJUST >> (%.1f cm)', rightWallDistance));
                brick.MoveMotor(rightMotor, SPEED * 0.75);  % gentler wall adjustment
                brick.MoveMotor(leftMotor, SPEED);
            end

        else
            % perfect - go straight
            disp(sprintf('  PERFECT === (heading %.1f°, wall %.1f cm)', headingError, rightWallDistance));
            brick.MoveMotor(rightMotor, SPEED);
            brick.MoveMotor(leftMotor, SPEED);
        end
    end

end

% cleanup - reset flipper and stop motors
disp('Resetting flipper motor...');
brick.MoveMotorAngleRel(flipperMotor, 50, 45, 'Coast');  % return to original position
pause(0.5);
brick.StopMotor(flipperMotor, 'Coast');  % ensure flipper can move freely
brick.StopMotor([rightMotor leftMotor], 'Coast');
CloseKeyboard();
disp('========================================');
disp('Maze mapping stopped.');
disp('========================================');
