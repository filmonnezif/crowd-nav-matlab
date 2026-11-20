function [vw, videoFile] = open_video(videoFile, frameRate)
%OPEN_VIDEO  Open a VideoWriter; MPEG-4 where available, else AVI.
%   [vw, file] = open_video('results/demo.mp4', 10)
%   MPEG-4 needs Windows or macOS; MATLAB Online (Linux) falls back to
%   Motion JPEG AVI, and the returned file name says which one was used.
%   vw is [] if no video can be written.
vw = [];
try
    vw = VideoWriter(videoFile, 'MPEG-4');
catch
    [d, n] = fileparts(videoFile);
    videoFile = fullfile(d, [n '.avi']);
    try
        vw = VideoWriter(videoFile, 'Motion JPEG AVI');
    catch err
        warning('open_video:video', 'No video: %s', err.message);
        vw = [];
    end
end
if ~isempty(vw)
    vw.FrameRate = frameRate;
    open(vw);
end
end
