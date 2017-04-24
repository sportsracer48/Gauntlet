rosinit('10.0.75.2',11311, 'NodeHost','10.0.75.1')
sub = rossubscriber('/stable_scan');

% Collect data at the room origin
scan_message = receive(sub);
r_1 = scan_message.Ranges(1:end-1);
theta_1 = [0:359]';

input('move the robot to a second location'); %Pause until you press a key
% Then collect data for the second location
scan_message = receive(sub);
r_2 = scan_message.Ranges(1:end-1);
theta_2 = [0:359]';

input('move the robot to a third location'); % Wait for key press
% Then collect data for the third location
scan_message = receive(sub);
r_3 = scan_message.Ranges(1:end-1);
theta_3 = [0:359]';

input('move the robot to a fourth location'); % Wait for key press

scan_message = receive(sub);
r_4 = scan_message.Ranges(1:end-1);
theta_4 = [0:359]';

% Gently shove everything into a matrix (you can use the matrix or the
% individual r_x and theta_x variables
r_all = [r_1 r_2 r_3 r_4];
theta_all = [theta_1 theta_2 theta_3 theta_4];