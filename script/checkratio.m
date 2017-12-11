%% Function for checking bwbwb ratio for qr identifier detection
% correct ratio = 1:1:3:1:1
% Pamir Ghimire, December 10, 2017

function ratiopositive = checkratio(b1, w1, b2, w2, b3)

ratiopositive = false;
input = [b1, w1, b2, w2, b3];
desired = [1, 1, 3, 1, 1];

input = input / min(input);
tolerance = 0.7;
if (norm(input - desired, 2) < tolerance)
    ratiopositive = true;
else
    ratiopositive = false;
end


end
