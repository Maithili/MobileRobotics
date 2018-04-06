function covariances_extract (landmark_nos)

% input = array of landmark ids in order
% output = square covariance matrix of robot pose followed by landmarks in
%          same order as the input

global State

if(iscolumn(landmark_nos))
    landmark_nos = landmark_nos';
end

complete_sigma = zeros(size(State.iSAM.R));
complete_sigma = 0./complete_sigma;

% indices in R of current robot x,y,theta and required landmarks
index_robot = [State.iSAM.rM(3*State.iSAM.nR-2), State.iSAM.rM(3*State.iSAM.nR-1), State.iSAM.rM(3*State.iSAM.nR)]
index_landmark=[];
for i=1:size(landmark_nos)
    index_landmark = [index_landmark, State.iSAM.lM(2*landmark_nos(i)-1),State.iSAM.lM(2*landmark_nos(i))];
end

inv_diag_R = 1./diag(State.iSAM.R);

% n_indices = length(indices);
% for i=1:n_indices
%     if(isnan(sigma(indices(i,1),indices(i,2))))
%         recover(indices(i,1),indices(i,2));
%     end
% end

minind = min([index_robot,index_landmark]);

complete_sigma = recover(minind, minind,complete_sigma,inv_diag_R);

State.iSAM.sigma = complete_sigma([index_robot,index_landmark]);

end
    
