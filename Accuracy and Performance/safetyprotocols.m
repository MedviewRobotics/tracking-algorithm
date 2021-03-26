function [xMicroscope, yMicroscope, zMicroscope] = safetyprotocols(xMicroscope_In, yMicroscope_In, zMicroscope_In)
%SAFETYPROTOCOLS Summary of this function goes here
%   Detailed explanation goes here

%Z from Tracking Output
if xMicroscope_In<35
     xMicroscope = 35;
elseif xMicroscope_In>115
     xMicroscope = 115;
else
     xMicroscope = xMicroscope_In;
end



%X from Tracking Output
if yMicroscope_In<-70
     yMicroscope = -70;
elseif yMicroscope_In>70
     yMicroscope = 70;
else
     yMicroscope = yMicroscope_In;
end

%Y from Tracking Output
if zMicroscope_In<100
     zMicroscope = 100;
elseif zMicroscope_In>170
     zMicroscope = 170;
else
     zMicroscope = zMicroscope_In;
end

end

