%% Geometric Model Class - GRAAL Lab
classdef geometricModel < handle
    % iTj_0 is an object containing the trasformations from the frame <i> to <i'> which
    % for q = 0 is equal to the trasformation from <i> to <i+1> = >j>
    % (see notes)
    % jointType is a vector containing the type of the i-th joint (0 rotation, 1 prismatic)
    % jointNumber is a int and correspond to the number of joints
    % q is a given configuration of the joints
    % iTj is  vector of matrices containing the transformation matrices from link i to link j for the input q.
    % The size of iTj is equal to (4,4,numberOfLinks)
    properties
        iTj_0
        jointType
        jointNumber
        iTj
        q
    end

    methods
        % Constructor to initialize the geomModel property
        function self = geometricModel(iTj_0,jointType)
            if nargin > 1
                self.iTj_0 = iTj_0;
                self.iTj = iTj_0;
                self.jointType = jointType;
                self.jointNumber = length(jointType);
                self.q = zeros(self.jointNumber,1);
            else
                error('Not enough input arguments (iTj_0) (jointType)')
            end
        end

        function updateDirectGeometry(self, q)

            for i=1:self.jointNumber
                Ti_0 = self.iTj_0(:,:,i);
                if self.jointType(i) == 0
                    theta = q(i); 
                    R = [cos(theta), -sin(theta), 0;
                         sin(theta),  cos(theta), 0;
                                0,           0,   1];
        
                    self.iTj(:,:,i) = Ti_0 * [R, [0; 0; 0]; 0, 0, 0, 1];
                end

                if self.jointType(i) == 1
                    transl = [0;0; q(i)];
                    self.iTj(:,:,i) = Ti_0 * [eye(3,3), transl; 0, 0, 0, 1];
                end
                    
            end
                          
            %%% GetDirectGeometryFunction
            % This method update the matrices iTj.
            % Inputs:
            % q : joints current position ;

            % The function updates:
            % - iTj: vector of matrices containing the transformation matrices from link i to link j for the input q.
            % The size of iTj is equal to (4,4,numberOfLinks)
            
            %TO DO
        end
        function [bTk] = getTransformWrtBase(self,k)

            bTk = eye(4);

            %SECONDO ME bTk matrice identità e il for va da 1 a k

            for i=1:k
                bTk = bTk  * self.iTj(:,:,i);
            end
           
            %% GetTransformatioWrtBase function
            % Inputs :
            % k: the idx for which computing the transformation matrix
            % outputs
            % bTk : transformation matrix from the manipulator base to the k-th joint in
            % the configuration identified by iTj.

            %TO DO
        end

    end
end


