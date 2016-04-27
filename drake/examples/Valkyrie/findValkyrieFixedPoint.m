function findValkyrieFixedPoint()

checkDependency('gurobi')

visualize = true;

% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

options.floating = true;
options.dt = 0.002;
r = Valkyrie('urdf/urdf/valkyrie_sim_drake.urdf',options);
r = r.removeCollisionGroupsExcept({'heel','toe', 'midfoot', 'feet'});
%r = r.removeCollisionGroupsExcept({'heel','toe', 'midfoot', 'left_hand', 'right_hand'});
%r = r.removeCollisionGroupsExcept({'left_hand', 'right_hand'});
r = compile(r);

nq = getNumPositions(r);
nu = r.getNumInputs();
v = r.constructVisualizer;

joints = r.getStateFrame.getCoordinateNames();
joints = joints(1:nq);
r_arm_joints = [find(strncmp(joints, 'rightShoulder', 13));
                find(strncmp(joints, 'rightElbowPitch', 13));  
                find(strncmp(joints, 'rightForearmYaw', 13));
                find(strncmp(joints, 'rightWrist', 10))];
l_arm_joints = [find(strncmp(joints, 'leftShoulder', 12));
                find(strncmp(joints, 'leftElbowPitch', 10));  
                find(strncmp(joints, 'leftForearmYaw', 10));
                find(strncmp(joints, 'leftWrist', 9))];
n_arm_joints = numel(r_arm_joints);

l_leg_joints = [find(strncmp(joints, 'leftHip', 7));
                find(strncmp(joints, 'leftKnee', 8));  
                find(strncmp(joints, 'leftAnkle', 9))];
r_leg_joints = [find(strncmp(joints, 'rightHip', 7));
                find(strncmp(joints, 'rightKnee', 8));  
                find(strncmp(joints, 'rightAnkle', 9))];
n_leg_joints = numel(r_leg_joints);

neck_joints = [find(strcmp(joints, 'lowerNeckPitch'));
               find(strcmp(joints, 'neckYaw'));
               find(strcmp(joints, 'upperNeckPitch'))];

scale = 0;
options.compl_slack = scale*0.01;
options.lincompl_slack = scale*0.001;
options.jlcompl_slack = scale*0.01;
% At the core this searches over poses, inputs, and joint states 
% to find a statically stable pose of the robot with constraints and
% costs we add
opt = ContactImplicitFixedPointUnconstrainedProgram(r.getManipulator(), [], options); 
opt = opt.setSolverOptions('snopt','DerivativeOption', 0);
opt = opt.setSolverOptions('snopt','VerifyLevel', 0);
opt = opt.setSolverOptions('snopt','MajorOptimalityTolerance', 1E-5);
opt = opt.setSolverOptions('snopt','SuperbasicsLimit', 10000);
opt = opt.setSolverOptions('snopt','print','snopt.out');

Q = 0.01*eye(nu);
c = zeros(nu, 1);
opt = opt.addInputCost(QuadraticSumConstraint(0,0,Q,c));

% remove floating base fredom that we don't need
% reasonable standing constraints:
opt = opt.addConstraint(ConstantConstraint(zeros(3,1)), opt.q_inds([1 2 6]));
opt = opt.addConstraint(ConstantConstraint(zeros(2,1)), opt.q_inds([4 5]));
opt = opt.addConstraint(BoundingBoxConstraint(1.01, 1.01), opt.q_inds([3]));
% zero out back joint
opt = opt.addConstraint(ConstantConstraint(zeros(3,1)), opt.q_inds([7, 8, 9]));

% some joint-specific tuning for practicality:
% zero out hip yaws:
opt = opt.addConstraint(ConstantConstraint(zeros(2,1)), opt.q_inds([find(strncmp(joints, 'leftHipYaw', 10)); find(strncmp(joints, 'rightHipYaw', 10))]));
% constrain arms joints:
l_arm_desired = [0, -1.0, 0.5, -1.57, 1.57, 0, 0.0].';
opt = opt.addConstraint(ConstantConstraint(l_arm_desired), opt.q_inds(l_arm_joints));
%r_arm_desired = [0, 0.7854, 1.5710, 0, 0, 0, 0].'; % inferred from symmetry
%opt = opt.addConstraint(ConstantConstraint(r_arm_desired), opt.q_inds(r_arm_joints));
% zero neck joints:
opt = opt.addConstraint(ConstantConstraint(zeros(3,1)),opt.q_inds(neck_joints));

%opt = opt.addCost(LinearConstraint(0, 0, -100), opt.q_inds([3]));

% symmetry in the arms
flip = [1 -1 1 -1 1 -1 1];
A = [eye(n_arm_joints) -diag(flip)];
opt = opt.addConstraint(LinearConstraint(zeros(n_arm_joints,1), zeros(n_arm_joints,1), A), [l_arm_joints;r_arm_joints]);

% symmetry in the legs
flip = [1 -1 1 1 1 -1];
A = [eye(n_leg_joints) -diag(flip)];
opt = opt.addConstraint(LinearConstraint(zeros(n_leg_joints,1), zeros(n_leg_joints,1), A), [l_leg_joints;r_leg_joints]);

% penalize distance of com from centroid of feet
% this is broken right now...
%const = FunctionHandleConstraint(0, 0, nq, @com_to_foot_centroid_constraint_fun);
%const.grad_method = 'numerical';
%opt = opt.addCost(const, {opt.q_inds});

% require contact force from feet
%opt = opt.addConstraint(BoundingBoxConstraint(zeros(opt.nC,1)+1, zeros(opt.nC,1)+Inf), opt.l_inds(1:(opt.nD+1):end));
%cp = 8;
%opt = opt.addCost(FunctionHandleConstraint(0, 0, nq, @feet_on_ground_constraint_fun), {opt.q_inds});

% set initial state to fixed point
xstar = 0;
%load('../../../../control/matlab/data/val_description/valkyrie_fp_gizatt_apr2016.mat');
xstar = zeros(nq*2, 1);
%xstar(1) = 0.1*randn();
%xstar(2) = 0.1*randn();
%xstar(6) = pi*randn();

guess = struct();
guess.q = double(xstar(1:nq));
v.draw(0, guess.q);
[q, u, l, info, F] = opt.findFixedPoint(guess, v);

function [f,df] = feet_on_ground_constraint_fun(q)
    colopts = opt.options.active_collision_options;
    colopts.terrain_only = true;
    [phi,normal,~,~,~,idxA,idxB,~,n,D,dn,dD] = r.getManipulator().contactConstraints(q,opt.options.multiple_contacts,colopts);
    f = 100*phi.'*phi;
    df = 200*phi.'*n;
    df(:,1:6) = 0;
end

function f = com_to_foot_centroid_constraint_fun(q)
    kin = r.doKinematics(q);
    compos = r.getCOM(q);
    
    % get chull of the support pts
    [phi,~,xA,xB,idxA,idxB] = r.collisionDetect(double(q));
    grndpts = [xA(1:2, idxA == 1) xB(1:2, idxB ==1)].';
    phis = [phi(idxA == 1); phi(idxB == 1)];
    chullptsinds = convhull(grndpts);
    chullpts = grndpts(chullptsinds, :);
    
    % get the gradients back
%     chullpts_grads = cell(1, 2);
%     for i=1:size(chullptsinds, 1)
%         if idxA(chullptsinds(i)) == 1
%             [~, J] = r.forwardKin(kin, idxB(chullptsinds(i)), xB(:, chullptsinds(i)));
%         else
%             [~, J] = r.forwardKin(kin, idxA(chullptsinds(i)), xA(:, chullptsinds(i)));
%         end
%         chullpts_grads{1} = [chullpts_grads{1}; double(J(1, :))];
%         chullpts_grads{2} = [chullpts_grads{2}; double(J(2, :))];
%     end
%     chullpts = TaylorVar(chullpts, chullpts_grads);
    
    % go over each edge and find min distance
    target = rand(1, 2);
    mindist = 1E9;
    for i=1:size(chullptsinds, 1)-1
        x = chullpts(i, :);
        y = chullpts(i+1, :);
        along = (y - x) / norm(y - x);
        outside = [-along(2) along(1)];
        dist = (target - y) * outside.';
        if dist < mindist
            mindist = dist;
        end
    end
    
    % calculate normal
    f = -0.01 * mindist;
end

end
