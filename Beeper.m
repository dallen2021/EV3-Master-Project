% beeper.m - beeps until touch sensor on port 1 is pressed
% authors: Daniel Allen, Nathan Alarcon, Thomas Stott, Ian Gallegos

display('push the button')

while brick.TouchPressed(1) == 0
    brick.playTone(100, 300, 500);
    pause(0.75);
end

display('done')
