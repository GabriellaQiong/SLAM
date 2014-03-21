function [indices] = sync_time(ts1, ts2)
% SYNC_TIME() synchronize time from different sensors
% Written by Qiong Wang at University of Pennsylvania
% 02/04/2014

% OUTPUT
% indices -- indices in time stamp1 with regard to time stamp 2

indices = zeros(numel(ts2), 1);
for i= 1 : numel(ts2)
    [~, idx]  = min(abs(ts2(i)-ts1));
    indices(i) = idx;
end

end