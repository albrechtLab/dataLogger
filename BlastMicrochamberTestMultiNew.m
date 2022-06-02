clc;

durin = 15; % ms in valve open
delay = [10];  % ms delay between in valve and out valve
durout = 15; % ms out valve open
%repeats = 20;
repeats = 10;

clear list

maxdata = 2000; % max data size
interval = 100; % us 
pre = 0; % us to record before start (when out1 activates)
post = 100000; % us to record after start

delaylist = repmat(1:length(delay), length(durin), 1); delaylist = delaylist(:);
durinlist = repmat(1:length(durin), length(delay), 1)'; durinlist = durinlist(:);

for l = 1:length(delaylist)
    list{l} = sprintf('%d,%d,%d',durin(durinlist(l)),delay(delaylist(l)),durout);
end

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
    %while (s.BytesAvailable > 0) fscanf(s); end
    flush(s);

    % set up acquisition settings
    disp("Setting Parameters:");
    cmd = ['i',num2str(interval)]; writeline(s, cmd); disp(cmd);
    pause(1);
    cmd = ['p',num2str(pre)]; writeline(s, cmd); disp(cmd);
    pause(1);
    cmd = ['P',num2str(post)]; writeline(s, cmd); disp(cmd);
    pause(1);
    disp("... complete");

    % example: send the same pulse X times
    for l = 1:length(list)
    
        disp(list{l});
    
        for i = 1:repeats

        % clear buffer
        %while (s.BytesAvailable > 0) fscanf(s); end
        flush(s);

        % start pulse
        %pause(0.2);
        disp(sprintf('[%s] Starting Arduino pattern %d of %d, repeat %d',datestr(now),l,length(list),i));
        writeline(s, list{l});
        pause(2);

        %disp(['Data received: ',num2str(s.BytesAvailable),' bytes']);

            % read & remove header
            while (s.BytesAvailable > 0 && ~contains(readline(s),'Logging'));
            end

            % read in data. 4 columns, comma separated
            data = [];
            while s.BytesAvailable > 0
                try 
                    data = [data; sscanf(readline(s), '%d,%d,%d,%d,%d,%d')'];
                catch 
                end
            end

            if isempty(data) data = NaN*ones(maxdata,6); end
            data(end+1:maxdata,:)=NaN;
            data = data(1:maxdata,:);

            % calculate time in us
            %t = [data(1,1); data(2:end,1)+1000*cumsum(data(2:end,1)<data(1:end-1,1))];
            t = data(:,1);

            v1 = data(:,2);
            v2 = data(:,3);
            v3 = data(:,4);
            dat1 = data(:,5);
            dat2 = data(:,6);
            %dat1 = ((data(:,5) / 1024) - 0.1)/0.8 * 150 ;  % conversion to psi
            %dat2 = ((data(:,6) / 1024) - 0.1)/0.8 * 150 ;  % conversion to psi

            maxdat = ceil(max(dat1) / 5) * 5;

            figure(1); clf; 
            plot(t/1000,[v1 * maxdat, v2 * maxdat, v3 * maxdat, dat1, dat2],'-'); 
            legend('Out1','Out2','Out3','Data1','Data2'); 
            xlabel('Time [ms]'); 
            ylabel('Pressure (psi)');

            % collect data into matrices
            if i==1 && l==1
                tmat = t;
                v1mat = v1;
                v2mat = v2;
                v3mat = v3;
                dat1mat = dat1;
                dat2mat = dat2;
                group = l;
            else
                tmat = [tmat, t];
                v1mat = [v1mat, v1];
                v2mat = [v2mat, v2];
                v3mat = [v3mat, v3];
                dat1mat = [dat1mat, dat1];
                dat2mat = [dat2mat, dat2];
                group = [group; l];
            end

        end
        %pause(1);
    end
end

% trim matrices
allnans = find(all(isnan(tmat),2));
if ~isempty(allnans)
    idx = 1:(min(allnans)-1);
    tmat = tmat(idx,:);
    v1mat = v1mat(idx,:);
    v2mat = v2mat(idx,:);
    v3mat = v3mat(idx,:);
    dat1mat = dat1mat(idx,:);
    dat2mat = dat2mat(idx,:);
end
%{
if (false)
    base = median(median(pmat));
    pmatcorr = pmat - base;
    pmean = mean(pmatcorr');
    pmax = max(pmean)
    pmin = min(pmean)
    range = find(pmean > (pmax/2));
    fwhm = t(range(end))-t(range(1))
    
    figure(2); clf;
    plot(t/1000, [mean(pmatcorr'); mean(vINmat'); mean(vOUTmat')]); xlabel('Time [ms]'); ylabel('Pressure [psi]');
    hold on; plot([t(range(1)) t(range(end))]/1000,[pmax/2, pmax/2]);
end
databrowseS(pmatcorr');
%}
    
