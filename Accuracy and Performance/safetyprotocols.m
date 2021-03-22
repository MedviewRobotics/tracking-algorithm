function [xMicroscope, yMicroscope, zMicroscope] = safetyProtocols(xMicroscope_In, yMicroscope_In, zMicroscope_In)
%SAFETYPROTOCOLS Summary of this function goes here
%   Detailed explanation goes here

%Z from Tracking Output
if xMicroscope_In<40
     xMicroscope = 40;
elseif xMicroscope_In>110
     xMicroscope = 110;
else
     xMicroscope = xMicroscope_In;
end



%X from Tracking Output
if yMicroscope_In<-50
     yMicroscope = -50;
elseif yMicroscope_In>50
     yMicroscope = 50;
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

