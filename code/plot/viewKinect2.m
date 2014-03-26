k       = load(fullfile(dataDir, ['kinect', num2str(dataIdx)]));
depthTs = k.depth_ts;
rgbTs   = k.rgb_ts;
jrgb    = k.jrgb;
zdepth  = k.zdepth;

t0Log  = min(depthTs(1),rgbTs(1));
tStart = GetUnixTime();

iDepth = 1;
iRgb   = 1;

hDepth = [];
hRgb   = [];

dt = 0;
update = 1;

while(1)
  dt = GetUnixTime() - tStart;
  
  if iDepth > length(depth_Ts)
      break;
  end
  
  if iRgb > length(rgbTs)
      break;
  end
  
  while(depthTs(iDepth)-t0Log < dt)
    iDepth = iDepth + 1;
    update = 1;
  end

  while(rgbTs(iRgb)-t0Log < dt)
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
