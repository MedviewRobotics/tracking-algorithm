%PREPROCESSFRAMES performs pre-processing on frame inputs from cameras.
%
%   [frame_left_out,frame_right_out] = preprocessFrames(frame_left_in,frame_right_in)
%   performs pre-processing on frame inputs from both the left and right
%   cameras. This includes converting to grayscale, smoothing, sharpening,
%   and spatial location filtering.
%
%   INPUTS:
%       frame_left_in = input frame from left camera
%       frame_right_in = input frame from right camera
%
%   OUTPUTS:
%       frame_left_out = frame from left camera after pre-processing
%       frame_right_out = frame from right camera after pre-processing

function [frame_left_out,frame_right_out] = preprocessFrames(frame_left_in,frame_right_in)

%Pre-process frames by converting to grayscale, smoothing, then sharpening
frame_left_out = rgb2gray(frame_left_in);
frame_left_out = imgaussfilt(frame_left_out);
frame_left_out = imsharpen(frame_left_out);

frame_right_out = rgb2gray(frame_right_in);
frame_right_out = imgaussfilt(frame_right_out);
frame_right_out = imsharpen(frame_right_out);

%Isolate Instrument Area
[M,N] = size(frame_left_out);

frame_left_out(1:M,[1:0.3*N 0.65*N:N],:)=0;
frame_left_out([1:200 550:M],1:N,:)=0;

frame_right_out(1:M,[1:0.3*N 0.65*N:N],:)=0;
frame_right_out([1:200 550:M],1:N,:)=0;

end

