%% Detection of qr identifiers in an image
% Pamir Ghimire
% December 10, 2017

clc; clear all; close all;
%%
img = imread('bwimage.png');
img = (img > 0);

%% State machine:
currState = 0;
b1 = 0; w1 = 0; b2 = 0; w2 = 0; b3 = 0;

startloc = [0, 0];
endloc = [0, 0];
detection = zeros(0, 2);

for nrow = 1:2:size(img, 1)
    % reset
    currState = 0;
    b1 = 0; w1 = 0; b2 = 0; w2 = 0; b3 = 0;
    pastFirstTest = false;
    
    r0 = [0, 0];
    r1 = [0, 0];
    
    for ncol = 1:size(img, 2)
        switch (currState)
            case 0 %no black encountered
                if (img(nrow, ncol) == 0)
                    currState = 1;
                    b1 = b1+1;
                end
                
            case 1 %inside first black
                if (img(nrow, ncol) == 1)
                    currState = 2;
                    w1 = w1 + 1;
                else
                    b1 = b1 + 1;
                end
                
            case 2 %inside first white
                if (img(nrow, ncol) == 0)
                    b2 = b2 + 1;
                    currState = 3;
                    r0 = [nrow, ncol];
                else
                    w1 = w1 + 1;
                end
                
            case 3 %inside second black
                if (img(nrow, ncol) == 1)
                    w2 = w2 + 1;
                    currState = 4;
                else
                    b2 = b2 + 1;
                end        
                
            case 4 %inside second white
                if (img(nrow, ncol) == 0)
                    b3 = b3 + 1;
                    currState = 5;
                    if(~pastFirstTest)
                        r1 = [nrow, ncol];
                    else
                        temp = r1;
                        r1 = [nrow, ncol];
                        r0 = temp;
                    end
                else
                    w2 = w2 + 1;
                end
                
            case 5 %inside third black
                if (img(nrow, ncol) == 1)
                    startloc = r0;
                    endloc = [nrow, ncol];

                    %fprintf(['---verifying\n', num2str(startloc), ' ', num2str(endloc), '\n']);
                    if (checkratio(b1, w1, b2, w2, b3))
                        %fprintf('detection!\n')
                        detection(end+1,:) = (startloc + endloc)/2;
                    end
                    
                    b1 = b2;
                    w1 = w2;
                    b2 = b3;
                    w2 = 1;
                    b3 = 0;
                    currState = 4;
                    if (~pastFirstTest)
                        pastFirstTest = true;
                    end
                    endloc = [0, 0];
                else
                    b3 = b3 + 1;
                end
        end
    end
end

%% display results
imshow(img), hold on
scatter(detection(:,2), detection(:,1), 'r')
title('detected qr identifiers in image rows');
