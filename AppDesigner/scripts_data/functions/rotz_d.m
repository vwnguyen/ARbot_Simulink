
function rot_z = rotz_d(ang)

    rot_z = [ cosd(ang) -sind(ang) 0;
            sind(ang) cosd(ang) 0 ;
            0 0 1;];
end