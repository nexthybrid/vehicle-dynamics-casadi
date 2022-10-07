% This program converts AVI video file into an animated GIF file.
% The GIF format have advantages especially in Power-Point presentation,
% and in internet browsers.
%
% Written by Moshe Lindner , Bar-Ilan University, Israel.
% September 2010 (C)
%
% Updated in November 2018, to use "VideoReader" instead of "aviread"
% and add options to resize and frop the frames

clear all
[file_name file_path]=uigetfile({'*.avi','AVI video files'},'Select Video file');
[file_name2 file_path2]=uiputfile('*.gif','Save as animated GIF',[file_path,file_name(1:end-3)]);

avi_file=VideoReader([file_path,file_name]);

lps=questdlg('How many loops?','Loops','Forever','None','Other','Forever');
switch lps
    case 'Forever'
        loops=65535;
    case 'None'
        loops=1;
    case 'Other'
        loops=inputdlg('Enter number of loops? (must be an integer between 1-65535)        .','Loops');
        loops=str2num(loops{1});
end

fps=avi_file.FrameRate;
N=avi_file.NumberOfFrames;
avi_file=VideoReader([file_path,file_name]);

delay=inputdlg('What is the delay time? (in seconds)        .','Delay',1,{num2str(1/fps)});
delay=str2num(delay{1});
dly=questdlg('Different delay for the first image?','Delay','Yes','No','No');
if strcmp(dly,'Yes')
    delay1=inputdlg('What is the delay time for the first image? (in seconds)        .','Delay');
    delay1=str2num(delay1{1});
else
    delay1=delay;
end
dly=questdlg('Different delay for the last image?','Delay','Yes','No','No');
if strcmp(dly,'Yes')
    delay2=inputdlg('What is the delay time for the last image? (in seconds)        .','Delay');
    delay2=str2num(delay2{1});
else
    delay2=delay;
end

rsz=questdlg('Resize the frames?','Resize','Yes','No','No');
if strcmp(rsz,'Yes');
    rs_fac=inputdlg('What is the resizing factor?','Resize',1);
    rs_fac=str2num(rs_fac{1});
end

crp=questdlg('Do you want to crop the frames?','Crop','Yes','No','No');
h = waitbar(0,['0% done'],'name','Progress') ;

for i=1:N
    
    a=readFrame(avi_file);
    if (i==1)&(strcmp(crp,'Yes'))
        fig=figure;
        imshow(a);
        title('Select area to crop');
        rect=round(getrect);
        close(fig);
        rect(rect<1)=1;
        if (rect(2)+rect(4))>size(a,1)
            rect(4)=size(a,1)-rect(2);
        end
        if (rect(1)+rect(3))>size(a,2)
            rect(3)=size(a,2)-rect(1);
        end
        X=rect(2)+[0:rect(4)];
        Y=rect(1)+[0:rect(3)];
    elseif (i==1)&(strcmp(crp,'No'))
        X=1:size(a,1);
        Y=1:size(a,2);
    end
    a=a(X,Y,:);
    if strcmp(rsz,'Yes');
        a=imresize(a,rs_fac);
    end
    [M  c_map]= rgb2ind(a,256);
    if i==1
        imwrite(M,c_map,[file_path2,file_name2],'gif','LoopCount',loops,'DelayTime',delay1)
    elseif i==N
        imwrite(M,c_map,[file_path2,file_name2],'gif','WriteMode','append','DelayTime',delay2)
    else
        imwrite(M,c_map,[file_path2,file_name2],'gif','WriteMode','append','DelayTime',delay)
    end
    waitbar(i/N,h,[num2str(round(100*i/N)),'% done']) ;
end
close(h);
msgbox('Finished Successfully!')