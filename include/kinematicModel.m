%% Kinematic Model Class - GRAAL Lab
classdef kinematicModel < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        gm % An instance of GeometricModel
        J % Jacobian
    end

    methods
        % Constructor to initialize the geomModel property
        function self = kinematicModel(gm)
            if nargin > 0
                self.gm = gm;
                self.J = zeros(6, self.gm.jointNumber);
            else
                error('Not enough input arguments (geometricModel)')
            end
        end

        function bJi = getJacobianOfLinkWrtBase(self, i)
            %%% getJacobianOfJointWrtBase
            % This method computes the Jacobian matrix bJi of joint i wrt base.
            % Inputs:
            % i : joint indnex ;

            % The function returns:
            % bJi

            
            T_0_i = self.gm.getTransformWrtBase(i);
            T_0_n = self.gm.getTransformWrtBase(length(self.gm.jointType));
            bJi = zeros(6,1);
            % Compute the Jacobian for joint i
            ki = T_0_i(1:3,3);
            r_ni = T_0_n(1:3,4) - T_0_i(1:3,4);
            %Angular part
            if(self.gm.jointType(i) == 0)
                
                w = ki;
            else
                % For prismatic joint
                w = [0,0,0]'; 
            end
            
            %Linear part
            if(self.gm.jointType(i) == 0)
                
                v = cross(ki, r_ni);
            else
                % For prismatic joint
                v = ki;
            end
         
            bJi(1:3) = w;
            bJi(4:6) = v;


        end

        function updateJacobian(self)
        %% Update Jacobian function
        % The function update:
        % - J: end-effector jacobian matrix

            for i = 1:self.gm.jointNumber
                Ji = self.getJacobianOfLinkWrtBase(i);
                self.J(:,i) = Ji;
            end
            
        end
    end
end

