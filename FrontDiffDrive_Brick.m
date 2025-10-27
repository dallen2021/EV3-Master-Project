% frontDiffDrive_Brick.m - differential drive control for EV3 robot
% authors: Daniel Allen, Nathan Alarcon, Thomas Stott, Ian Gallegos

function FrontDiffDrive_Brick(brick, leftPort, rightPort, invertDrive)

if nargin<2, leftPort=1; end
if nargin<3, rightPort=3; end
if nargin<4, invertDrive=true; end

minStraight = 25;  maxStraight = 90;  accelStraight = 80;  % %/s
minTurn     = 30;  maxTurn     = 80;  accelTurn     = 70;  % %/s
rateHz  = 40;  deadman = 0.25;  brakeMode = 'Brake';

cmd = "none";           % "fwd","rev","left","right","none"
tHold = tic;            % time since current cmd started
tLastKey = tic;         % time since any key was down

global key
InitKeyboard();
c = onCleanup(@()shutdown());
fprintf('Arrows: hold-to-accelerate. Space=brake. +/- caps. Q=quit.\n');

dt = 1/rateHz;
while true
    pause(dt);

    prev = cmd;
    switch key
        case 'downarrow',   cmd = iff(invertDrive,"rev","fwd");
        case 'uparrow',     cmd = iff(invertDrive,"fwd","rev");
        case 'rightarrow',  cmd = "left";
        case 'leftarrow',   cmd = "right";
        case 'space',       cmd = "none"; stopAll(); fprintf("Brake\n");
        case {'add','equal','+'}
            maxStraight = min(100, maxStraight+5);
            maxTurn     = min(100, maxTurn+5);
            fprintf('Caps -> straight:%d turn:%d\n', maxStraight, maxTurn);
        case {'subtract','hyphen','-'}
            maxStraight = max(10,  maxStraight-5);
            maxTurn     = max(10,  maxTurn-5);
            fprintf('Caps -> straight:%d turn:%d\n', maxStraight, maxTurn);
        case {'q','escape'}, break
        otherwise           % no new key event
            % keep current cmd
    end

    % reset acceleration timer ONLY when the command changes
    if cmd ~= prev
        tHold = tic;
        if cmd=="fwd",  fprintf('Forward\n'); end
        if cmd=="rev",  fprintf('Reverse\n'); end
        if cmd=="left", fprintf('Turn Left\n'); end
        if cmd=="right",fprintf('Turn Right\n'); end
    end

    if ~isequal(key,0)
        tLastKey = tic;
    end

    if (cmd=="none") || (isequal(key,0) && toc(tLastKey) > deadman)
        stopCoast(); 
        continue
    end

    % speed from hold time while same arrow is held
    hold = toc(tHold);  % seconds
    switch cmd
        case {"fwd","rev"}
            sp = clamp(minStraight + accelStraight*hold, 0, maxStraight);
            v  = iff(cmd=="fwd", +sp, -sp);
            brick.MoveMotor(leftPort,  v);
            brick.MoveMotor(rightPort, v);

        case {"left","right"}
            sp  = clamp(minTurn + accelTurn*hold, 0, maxTurn);
            sgn = iff(cmd=="right", +1, -1);   % right: L back, R fwd
            brick.MoveMotor(leftPort,  clamp(-sgn*sp, -100, 100));
            brick.MoveMotor(rightPort, clamp(+sgn*sp, -100, 100));
    end
end

CloseKeyboard();
shutdown();

function o = iff(c,a,b), if c, o=a; else, o=b; end, end
function y = clamp(x,lo,hi)
    if nargin<2, lo=-100; end
    if nargin<3, hi=100;  end
    y = max(lo, min(hi, round(x)));
end
function stopAll(),   try, brick.StopMotor([leftPort rightPort],brakeMode); catch, end, end
function stopCoast(), try, brick.StopMotor([leftPort rightPort],'Coast');   catch, end, end
function shutdown(),  stopCoast(); end
end
