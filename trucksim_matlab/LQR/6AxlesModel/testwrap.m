function val = testwrap(angle)
    if(angle  > pi)
        angle = angle - 2*pi;
    elseif (angle  < -pi)
        angle = angle + 2*pi;
    else
        angle = angle;
    end
    val = angle;
end