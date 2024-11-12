% Define constants
B = 0.76;
C = 20;
D = 72.28427124746186;

% Initialize variables to store results
A_values = zeros(1, 350);
H_values = 1:350;

% Loop over each value of H
for H = H_values
    % Calculate E
    E = 12 + ceil(H / 50) * 2;
    
    % Determine F and I based on H
    if H < 300
        F = 2000;
        I = 2;
    else
        F = 2500;
        I = 4;
    end
    
    % Calculate G
    G = ceil(3000 / H);
    
    % Calculate A
    A = (B * C * I * D + E * D + F) * G;
    
    % Store the result
    A_values(H) = A;
end

% Find the minimum value of A and the corresponding H
[min_A, min_H_index] = min(A_values);
min_H = H_values(min_H_index);

% Display the results
fprintf('The minimum value of A is %.2f at H = %d\n', min_A, min_H);

figure;
plot(H_values, A_values);
xlabel('H');
ylabel('A');
title('Relation between A and H');
grid on;