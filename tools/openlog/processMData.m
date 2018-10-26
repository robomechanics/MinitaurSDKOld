close all; clear; clc;

mData = open_log('LOG00006.txt');

% Reformat user params
N = length(mData.UP);
for i = 1:N
    mData.params(i,1) = typecast(uint8(mData.UP(i,1:4)), 'single');
    mData.params(i,2) = typecast(uint8(mData.UP(i,5:8)), 'single');
    mData.params(i,3) = typecast(uint8(mData.UP(i,9:12)), 'single');
    mData.params(i,4) = typecast(uint8(mData.UP(i,13:16)), 'single');
    mData.params(i,5) = typecast(uint8(mData.UP(i,17:20)), 'single');
    mData.params(i,6) = typecast(uint8(mData.UP(i,21:24)), 'single');
    mData.params(i,7) = typecast(uint8(mData.UP(i,25:28)), 'single');
    mData.params(i,8) = typecast(uint8(mData.UP(i,29:32)), 'single');
end

% Rescale time
mData.t = (mData.t - mData.t(1))*0.001;

% Identify wrong data to omit
c = struct2cell(mData);
tClear = [];
for t = 1:N
    for val = 1:length(c)
        if max(abs(c{val}(t,:))) > 9999
            tClear = [tClear; t];
            break;
        end
    end
end

% Trim wrong data
for i = 1:length(c)
    while length(c{i})>N
        c{i}(end,:) = [];
    end
    
    for t = flipud(tClear)
        c{i}(t,:) = [];
    end
end

N = length(c{1});
mDataOld = mData;
mData = cell2struct(c, fieldnames(mData), 1);

mData.vel = mData.params(:,4);

plot(mData.t, mData.vel)