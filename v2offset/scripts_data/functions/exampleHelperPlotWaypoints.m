function exampleHelperPlotWaypoints(wayPoints)
% This function plots spheres as points of interest
% wayPoints is an is an array with rows [ x y z ] in 3d space
% Copyright 2017-2018 The MathWorks, Inc.

for idx = 1:size(wayPoints,1)
   exampleHelperPlotSpheres(0.025,wayPoints(idx,:));
end

