t0Log  = min(depth_ts(1),rgb_ts(1));
tStart = GetUnixTime();

iDepth = 1;
iRgb   = 1;

hDepth = [];
hRgb   = [];

dt = 0;
update = 1;

while(1)
  dt = GetUnixTime() - tStart;
  
  while(depth_ts(iDepth)-t0Log < dt)
    iDepth = iDepth + 1;
    update = 1;
  end

  while(rgb_ts(iRgb)-t0Log < dt)
    iRgb = iRgb + 1;
    update = 1;
  end
  
  if update == 1
    depth = reshape(typecast(zlibUncompress(zdepth{iDepth}),'uint16'),[640 480])';
    rgb   = djpeg(jrgb{iRgb});

    if isempty(hDepth)
      figure(1), clf(gcf);
      hDepth = imagesc(depth);
    else
      set(hDepth,'cdata',depth);      
    end

    if isempty(hRgb)
      figure(2), clf(gcf);
      hRgb = image(rgb);
    else
      set(hRgb,'cdata',rgb);
    end
  
    drawnow;
  end
  
  update = 0;
  
  pause(0.05);
end
