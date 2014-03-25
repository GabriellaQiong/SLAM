k = load(fullfile(dataDir, ['kinect', num2str(dataIdx)]));
zdepth = k.zdepth;
depth_ts = k.depth_ts;
len = length(zdepth);
depths = cell(1,len);

for ii=1:len
  depths{ii} = reshape(typecast(zlibUncompress(zdepth{ii}),'uint16'),[640 480]);
end

splitSize = 500;
nSplit = ceil(len/splitSize);

for jj=1:nSplit
  fName  = sprintf('kinect%d_depth_%d',dataIdx,jj);
  inds = (jj-1)*splitSize+1:jj*splitSize;
  if (inds(end) > len)
    inds = (jj-1)*splitSize+1:len;
  end
  udepth = depths(inds);
  udepth_ts = depth_ts(inds);
  
  save(fullfile(dataDir, fName),'udepth','udepth_ts');
  fprintf('saved split number %d\n',jj);
end
