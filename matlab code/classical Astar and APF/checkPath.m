function feasible=checkPath(n,newPos,shp)
feasible=true;
movingVec = [newPos(1)-n(1),newPos(2)-n(2),newPos(3)-n(3)];
movingVec = movingVec/sqrt(sum(movingVec.^2)); %单位化
if n==newPos  %%防止对于同一个点检测
    feasible=false;
else
    for R=0:0.1:sqrt(sum((n-newPos).^2))
    posCheck=n + R .* movingVec;
    tf = inShape(shp,posCheck(:,1),posCheck(:,2),posCheck(:,3));
      if (tf==1)
        feasible=false;break;
      end
    end
end

