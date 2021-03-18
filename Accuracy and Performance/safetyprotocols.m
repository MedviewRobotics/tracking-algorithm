function [xMicroscope, yMicroscope, zMicroscope] = safetyprotocols(xMicroscope_In, yMicroscope_In, zMicroscope_In)
%SAFETYPROTOCOLS Summary of this function goes here
%   Detailed explanation goes here

%Z from Tracking Output
if xMicroscope_In<50
     xMicroscope = 50;
elseif xMicroscope_In>100
     xMicroscope = 100;
else
     xMicroscope = xMicroscope_In;
end



%X from Tracking Output
if yMicroscope_In<-45
     yMicroscope = -45;
elseif yMicroscope_In>45
     yMicroscope = 4;
else
     yMicroscope = yMicroscope_In;
end

%Y from Tracking Output
if zMicroscope_In<110
     zMicroscope = 110;
elseif zMicroscope_In>150
     zMicroscope = 150;
else
     zMicroscope = zMicroscope_In;
end

end

