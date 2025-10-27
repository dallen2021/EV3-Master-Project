% colorTest.m - test color detection functionality
% demonstrates stopping behavior for red, blue, and green colors
% authors: Daniel Allen, Nathan Alarcon, Thomas Stott, Ian Gallegos

% sensor ports
colorPort = 1;      % color sensor

% motor ports
rightMotor = 'A';
leftMotor = 'C';

% navigation parameters
SPEED = 30;

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

% initialize keyboard for exit
global key
InitKeyboard();

disp('========================================');
disp('COLOR DETECTION TEST');
disp('========================================');
disp('Robot will respond to colors:');
disp('  RED: Stop for 1 second');
disp('  BLUE: Stop and beep 2 times');
disp('  GREEN: Stop and beep 3 times');
disp(' ');
disp('Press Q to stop');
disp('========================================');
pause(2);

disp('Starting color detection test...');

while key ~= 'q'
    % read color sensor
    detectedColor = brick.ColorCode(colorPort);

    % display current color
    disp(sprintf('Color code: %d', detectedColor));

    % check for specific colors
    if detectedColor == COLOR_RED
        disp('>>> RED DETECTED! Stopping for 1 second...');
        brick.StopMotor([rightMotor leftMotor], 'Brake');
        pause(1);
        disp('    Resuming...');

    elseif detectedColor == COLOR_BLUE
        disp('>>> BLUE DETECTED! Stopping and beeping 2 times...');
        brick.StopMotor([rightMotor leftMotor], 'Brake');
        brick.beep();
        pause(0.5);
        brick.beep();
        pause(0.5);
        disp('    Resuming...');

    elseif detectedColor == COLOR_GREEN
        disp('>>> GREEN DETECTED! Stopping and beeping 3 times...');
        brick.StopMotor([rightMotor leftMotor], 'Brake');
        brick.beep();
        pause(0.5);
        brick.beep();
        pause(0.5);
        brick.beep();
        pause(0.5);
        disp('    Resuming...');

    else
        % no color detected or different color - keep moving
        brick.MoveMotor(rightMotor, SPEED);
        brick.MoveMotor(leftMotor, SPEED);
    end

    pause(0.1);  % small delay between readings
end

% cleanup - stop motors and close keyboard
brick.StopMotor([rightMotor leftMotor], 'Coast');
CloseKeyboard();
disp('========================================');
disp('Color detection test stopped.');
disp('========================================');
