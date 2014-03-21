function [startIdx] = sync_time(ts1, ts2)
% SYNC_TIME() synchronize time from different sensors
% Written by Qiong Wang at University of Pennsylvania
% 02/04/2014

% OUTPUT
% startIdx -- start index of time according to second sensor, if positive
%             time starts from ts1(startIdx), if negative time starts from
%             ts2(-startIdx)

% Compute the differences
diff1 = abs(ts1 - ts2(1));
diff2 = abs(ts2 - ts1(1));

% Find the smallest difference point
[val1, idx1] = min(diff1);
[val2, idx2] = min(diff2);
startIdx = (val1 < val2) * idx1 - (val1 >= val2) * idx2;

end