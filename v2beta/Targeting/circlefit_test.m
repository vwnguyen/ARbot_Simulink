p1=rand(4,3);p2=rand(4,3);p3=rand(4,3);
[center,rad,v1,v2] = circlefit3d(p1,p2,p3);
plot3(p1(:,1),p1(:,2),p1(:,3),'bo');hold on;plot3(p2(:,1),p2(:,2),p2(:,3),'bo');plot3(p3(:,1),p3(:,2),p3(:,3),'bo');
for i=1:361,
a = i/180*pi;
x = center(:,1)+sin(a)*rad.*v1(:,1)+cos(a)*rad.*v2(:,1);
y = center(:,2)+sin(a)*rad.*v1(:,2)+cos(a)*rad.*v2(:,2);
z = center(:,3)+sin(a)*rad.*v1(:,3)+cos(a)*rad.*v2(:,3);
plot3(x,y,z,'r.');
end
axis equal;grid on;rotate3d on; 