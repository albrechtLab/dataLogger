clc;

interval = 2000-5; % us per data point 
recordSec = 10; % seconds of data to record

% check for existing serial object. if not, make it 
if ~exist('s','var')
    port = serialportlist;
    %port = {'COM9'}; % Calcium 2 COM port
    
    if (length(port) >= 1) 
        s = serialport(port(end),500000,'Parity','none');
        %s.InputBufferSize = 4096;
        configureTerminator(s,'LF'); %s.Terminator = 'LF';
        %s.Parity = 'even';
    else
        error('No Serial ports found');
    end
else 
    %fclose(s);
    %pause(1);
end

% Open serial port if not already open
if exist('s','var')
    if strcmp(class(s), 'internal.Serialport')
        if ~strcmp(s.Status, 'open') 
            fopen(s);
            disp(['Serial port opened: ',s.Port]);
        else
            disp(['Serial port already open: ',s.Port]);
        end
    end
end

if strcmp(s.Status, 'open') 
    
    % clear buffer
    flush(s);

    % set up acquisition settings
    disp("Setting Parameters:");
    cmd = ['i',num2str(interval)]; writeline(s, cmd); disp(cmd);
    pause(1);
    disp("... complete");
    
    fprintf("\nReady to begin recording for %d seconds at %f kHz (target).\n",recordSec,1000/interval); 
    fprintf("Press a key or mouse button to begin data logging.\n\n");
    w = waitforbuttonpress;

    fprintf('[%s] Begin recording.\n',datestr(now));    
    writeline(s, ['R' num2str(recordSec)]);
    tic;

%     fprintf('Time Remaining (s):    ');
%     for tr = recordSec:-1:1
%         fprintf('\b\b\b%3d',tr);
%         pause(1);
%     end
%     fprintf('\n[%s] Recording complete\n', datestr(now));
%     pause(3);
    pause(3);

    % read & remove header
    while (s.BytesAvailable > 0 && ~contains(readline(s),'Logging'));
    end

    fprintf('[%s] Reading data\n', datestr(now));
    % read in data. 4 columns, comma separated
    data = []; i=0;
    while s.BytesAvailable > 0
        try 
            data = [data; sscanf(readline(s), '%d,%d,%d,%d,%d,%d')'];
            i = i+1;
            if (mod(i,100) == 0) fprintf("X"); end
            if (mod(i,1000) == 0) fprintf("|"); end
            if (mod(i,5000) == 0) fprintf(" %d\n",i); end

            if ((s.BytesAvailable == 0) && (toc < recordSec)) pause(1); end 
        catch 
        end
    end

    fprintf("\n%d data points recorded in %0.6f seconds at %f kHz.\n", i, data(end,1)/1000000, 1000 / (data(end,1) / i));

    figure(1); plot(data(:,1),data(:,2:6));
end

