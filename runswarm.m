function [history_Z,history_position] = runswarm(r, iter, Nz, k0, k1, k2, V0)
    
    N0 = agent(r(1,:),V0);
    N1 = agent(r(2,:),V0);
    N2 = agent(r(3,:),V0);
    N3 = agent(r(4,:),V0);
    
    history_Z = zeros(2,Nz,iter);
    history_position = zeros(iter,Nz,3);
    
    zd = 3;

    for i = 1:iter
        
        % observation topology
        N0_nbr = [N1.XY;N2.XY;N3.XY];
        N1_nbr = [N0.XY;N2.XY;N3.XY];
        N2_nbr = [N1.XY;N0.XY;N3.XY];
        N3_nbr = [N1.XY;N2.XY;N0.XY];


        N0 = N0.run(N0_nbr,zd,i,k0,k1,k2,0);
        N1 = N1.run(N1_nbr,zd,i,k0,k1,k2,1);
        N2 = N2.run(N2_nbr,zd,i,k0,k1,k2,2);
        N3 = N3.run(N3_nbr,zd,i,k0,k1,k2,3);


        history_Z(1,1,i) = N0.Z;
        history_Z(1,2,i) = N1.Z;
        history_Z(1,3,i) = N2.Z;
        history_Z(1,4,i) = N3.Z; 
        
        history_Z(2,1,i) = N0.XY(1,3);
        history_Z(2,2,i) = N1.XY(1,3);
        history_Z(2,3,i) = N2.XY(1,3);
        history_Z(2,4,i) = N3.XY(1,3); 

        history_position(i,1,:) = N0.XY;
        history_position(i,2,:) = N1.XY;
        history_position(i,3,:) = N2.XY;
        history_position(i,4,:) = N3.XY;

    end

end