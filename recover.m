function s = recover (i,l,s,inv_R)

global State

sum = 0;
for j=1:length(s)
    if (State.iSAM.R(i,j)~=0 && j~=i)
        if (j>l)
          if (isnan(s(l,j)))
              recover(l, j, s, inv_R);
          end
          sum = sum + State.iSAM.R(i,j) * s(l,j);
        else
          if (isnan(s(j,l)))
              recover(j, l, s, inv_R);
          end
          sum = sum + State.iSAM.R(i,j) * s(j,l);
        end
    end
end

if (i == l)                                 % diagonal entries
    s(i,l) =  (inv_R(l) * (inv_R(l) - sum));
else                                        % off-diagonal entries
    s(i,l) = (- sum * inv_R(i));
    s(l,i) = s(i,l);
end
