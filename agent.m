classdef agent
   properties
      XY %当前坐标
      Z  %当前标量场Value
      V  %速度
      v0_0 
      v1_0
      v2_0
      dt
      iter
      fd
      wd
   end
   methods
        function veh = agent(XY0,V0)
            veh.XY = XY0;
            veh.V = zeros(1,3);
            veh.v2_0 = V0(3,:)';
            veh.v0_0 = V0(1,:)';
            veh.v1_0 = V0(2,:)';
            veh.dt = 0.00005;
            tic
            veh.Z = getfield(veh,1);
            toc
            veh.iter = 42000;
            veh.fd = 0;
            veh.wd = 0.1;
        end

        function Z = getfield(veh,t)
            Z = tv_field(veh.XY(1,1),veh.XY(1,2),veh.XY(1,3),t);
        end

        function out = update(veh,u)

%             w = veh.flow(veh.XY);
            veh.V = u;

            veh.XY(1,1) = veh.XY(1,1) + veh.V(1,1)*veh.dt;
            veh.XY(1,2) = veh.XY(1,2) + veh.V(1,2)*veh.dt;
            veh.XY(1,3) = veh.XY(1,3) + veh.V(1,3)*veh.dt;
                       
            out = veh; 
        end

        function V = run(veh,nbr,zd,times,k0,k1,k2,testdiff)

            [u,out] = getU(veh,nbr,zd,k0,k1,k2,times,testdiff);
            V = out.update(u);
           
        end
    
        function [U,out] = getU(veh,nbr,zd,k0,k1,k2,times,testdiff)
            %等比例缩放
            
            r = [veh.XY;nbr];

            % observe the relative positions 
            relative_r = r - veh.XY;

            [coeff,~,~] = pca(relative_r,'Algorithm','svd','NumComponents',3);

            v2 = coeff(:,1)./norm(coeff(:,1));
            v1 = coeff(:,2)./norm(coeff(:,2));
            v0 = coeff(:,3)./norm(coeff(:,3));

            % continuous PCA
%             e = 0.01;
%             [t,q] = ode45(@(t,q) qode(q,C,e), [0,0.1],[1;0]);
%             v2 = q(end,:)';
            
            
            % make sure vi(t-1)*vi(t) > 0
            if veh.v2_0'*v2 < 0
                v2 = -v2;
            end
            if veh.v0_0'*v0 < 0
                v0 = -v0;
            end
            if veh.v1_0'*v1 < 0
                v1 = -v1;
            end
            
            veh.v0_0 = v0;
            veh.v1_0 = v1;
            veh.v2_0 = v2;

            veh.Z = veh.getfield(times);
            delta_Z = veh.Z-zd;
            
            ri = mean(relative_r);
            rcj = relative_r - mean(relative_r);
            dc = max(sqrt(rcj(:,1).^2+rcj(:,2).^2+rcj(:,3).^2));

            if dc > 0.03
                kc = 1.5*k0;
            else 
                kc = 0;
            end

            % continuous Kc
%             gamma = 0.2;
%             rho = 40;
%             kc = 0.75*k0*(tanh(rho*(norm(ri)-gamma))+1);
            
            switch testdiff
                case 0
                    veh.fd = veh.XY(1,3)-0.896;
                    U = (k0*(delta_Z).*v0+k1.*v2+k2*(veh.fd).*v1)'+kc*ri;
                case 1
                    veh.fd = veh.XY(1,3)-0.898;
                    U = (k0*(delta_Z).*v0+k1.*v2+k2*(veh.fd).*v1)'+kc*ri;
                case 2
                    veh.fd = veh.XY(1,3)-0.902;
                    U = (k0*(delta_Z).*v0+k1.*v2+k2*(veh.fd).*v1)'+kc*ri;
                case 3
                    veh.fd = veh.XY(1,3)-0.904;
                    U = (k0*(delta_Z).*v0+k1.*v2+k2*(veh.fd).*v1)'+kc*ri;
                otherwise
                    veh.fd = veh.XY(1,3)-0.85+0.05*sin(3*pi*times/(veh.iter));
                    U = (k0*(delta_Z).*v0+k1.*v2+k2*(veh.fd).*v1)'+kc*ri;
            end
            
            out = veh;
            end
        end  
       
end