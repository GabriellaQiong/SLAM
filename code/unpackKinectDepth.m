
len = length(zdepth);
depths = cell(1,len);

for ii=1:len
  depths{ii} = reshape(typecast(zlibUncompress(zdepth{ii}),'uint16'),[640 480]);
end

splitSize = 500;
nSplit = ceil(len/splitSize);

dataSet = 20;

for jj=1:nSplit
  fName  = sprintf('kinect%d_depth_%d',dataSet,jj);
  inds = (jj-1)*splitSize+1:jj*splitSize;
  if (inds(end) > len)
    inds = (jj-1)*splitSize+1:len;
  end
  udepth = depths(inds);
  udepth_ts = depth_ts(inds);
  
  save(fName,'udepth','udepth_ts');
  fprintf('saved split number %d\n',jj);
end
