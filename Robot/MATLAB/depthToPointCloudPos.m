function pos = depthToPointCloudPos(x, y, depth)
%     fx_d = 1.0 / 5.9421434211923247e+02;
%     fy_d = 1.0 / 5.9104053696870778e+02;
%     cx_d = 3.3930780975300314e+02;
%     cy_d = 2.4273913761751615e+02;

    fx_d = 365.456;
    fy_d = 365.456;
    cx_d = 254.878;
    cy_d = 205.395;
    
    pz = depth/1000;
    px = (x - cx_d)*pz/fx_d;
    py = (y - cy_d)*pz/fy_d;
    pos = [px,py,pz];
end

