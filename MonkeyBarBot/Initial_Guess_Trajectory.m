theta2_guess = [0, pi/10, pi/10, 0, -pi/10, -2*pi/10, ...
                -3*pi/10, -2*pi/10, -pi/10, 0, pi/10, ...
                0, -pi/10, -2*pi/10, 0, 2*pi/10, 3*pi/10, 1.4455];
theta1_guess = [0, 0, pi/20, pi/10, pi/10, pi/20, ...
                -pi/20, -pi/10, -3*pi/20, -4*pi/20, ...
                -4*pi/20, -3*pi/20, -2*pi/20, -pi/20, ...
                0, pi/20, 3*pi/20, 0.8481];

theta1guess = [];
for i = 1:(length(theta1_guess)-1)
    theta1guess = [theta1guess, theta1_guess(i), (theta1_guess(i) + theta1_guess(i+1))/2];
end
theta1guess = [theta1guess, theta1_guess(end)]

theta2guess = [];
for i = 1:(length(theta2_guess)-1)
    theta2guess = [theta2guess, theta2_guess(i), (theta2_guess(i) + theta2_guess(i+1))/2];
end
theta2guess = [theta2guess, theta2_guess(end)]

fprintf("[");
for i = 1:length(theta1guess)
    if (i ~= 1) 
        fprintf(", ")
    end
    fprintf("%d", theta1guess(i))
end
fprintf("]\n");
    
fprintf("[");
for i = 1:length(theta2guess)
    if (i ~= 1) 
        fprintf(", ")
    end
    fprintf("%d", theta2guess(i))
end
fprintf("]\n");