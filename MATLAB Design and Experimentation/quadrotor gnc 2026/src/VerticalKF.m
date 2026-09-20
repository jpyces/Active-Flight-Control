classdef VerticalKF < handle
    properties
        x           % 3x1 state: [altitude; velocity; accelBias]
        P           % 3x3 covariance
        Q           % 3x3 process noise (fixed, tuned once via synthetic/static data)
        R           % scalar measurement noise (fixed, tuned from static baro variance)
        initialized
    end

    properties (Constant)
        H = [1 0 0]     % measurement matrix never changes — keep it explicit, not rebuilt each call
    end

    methods
        function obj = VerticalKF(Q, R, x0, P0)
            % store Q, R
            obj.Q = Q; obj.R = R;
            obj.x = x0; obj.P = P0; obj.initialized = true;
        end

        function predict(obj, aMeas, dt)
            % build A (3x3) and B (3x1) from dt — see the F/B matrices worked out earlier
                   % z_k + v_k*dt + 0.5(u_k - b_k)*dt^2
                   % v_(k+1) = v_k + (u_k - b_k)*dt
                   % b_(k+1) = b_k
            % A - akin to a dynamics model - determines what would happen
            %   when there is no input at all - updating positions and
            %   velocity
            A = [
                     1  dt  -.5*dt^2; 
                     0  1   -dt; 
                     0  0   1           % bias persists across time steps

                ];  % State transition matrix
            
            % B is the input/control part "../../../MATLAB copy/quadrotor gnc 2026/src"- the external input quantity
            %   0.5*dt^2 is the accel term and dt worth of velocity change
            %   from aMeas
            B = [
                    0.5*dt^2; 
                    dt; 
                    0           % accel input shouldn't touch the bias state
                ];          % Control input matrix
            
            % Basically, what happens to the state naturally and what the
            %   input does - in this case, how the changing accel aMeas
            %   affects the dynamics of the state of the system
            obj.x = A*obj.x + B*aMeas;

            % Priori Covariance Predict
            obj.P = A*obj.P*A' + obj.Q;
        end

        function correct(obj, zMeas)
            % obj.H*obj.P*obj.H' collapses full 3x3 covariance to a scalar
            % for P(1,1) to get the true noise for altitude in this setup
            S = obj.H*obj.P*obj.H' + obj.R; % measurement innovation aka residual Covariance
            K = obj.P*obj.H' / S;

            % This is the line doing the real work, and it's worth sitting 
            %   with. P*H' is 3x1 — and here's the key thing: even though H 
            %   only directly observes altitude, P*H' pulls out the first 
            %   column of P, meaning it captures how correlated velocity 
            %   and bias currently are with altitude, via P's off-diagonal 
            %   terms — not just P(1,1) alone. Dividing that 3x1 vector by 
            %   the scalar S gives you K, a 3x1 vector: 
            %   "how much should altitude, velocity, and bias each move, 
            %   per unit of residual." 

            % If model's uncertainty about altitude (P(1,1)) is large 
            %   relative to R, K(1) comes out large — trust the new 
            %   measurement heavily. If R dominates (noisy barometer), 
            %   K shrinks — trust your prediction more. 
            %   Same logic, scaled by covariance, governs K(2) and K(3).
            obj.x = obj.x + K*(zMeas - obj.H*obj.x);

            % Updating the error state covariance
            % This shrinks the covariance to reflect that just 
            %   incorporated real information — the amount of shrinkage is 
            %   proportional to K, so a confident correction (large K) 
            %   shrinks P more than a barely-trusted one does.

            % simple form - obj.P = (eye(3) - K*obj.H)*obj.P;
            
            % robust form:
            I = eye(3);
            obj.P = (I - K*obj.H)*obj.P*(I - K*obj.H)' + K*obj.R*K';
        end

        function reset(obj, x0, P0)
            % same role reset() plays on your other controllers: called at a state-machine transition
            obj.x = x0;
            obj.P = P0;
         
        end
    end
end