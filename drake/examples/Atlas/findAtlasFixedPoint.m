function findAtlasFixedPoint()

checkDependency('gurobi')

visualize = true;

% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

options.floating = true;
options.dt = 0.002;
r = Atlas('/home/gizatt/drc/software/drake/drake/examples/Atlas/urdf/atlas_convex_hull.urdf',options);
r = r.removeCollisionGroupsExcept({'heel','toe'});
r = compile(r);

nq = getNumPositions(r);
nu = r.getNumInputs();
v = r.constructVisualizer;

joints = r.getStateFrame.getCoordinateNames();
joints = joints(1:nq);
l_arm_joints = find(strncmp(joints, 'l_arm_', 5));
r_arm_joints = find(strncmp(joints, 'r_arm_', 5));
n_arm_joints = numel(r_arm_joints);

l_leg_joints = find(strncmp(joints, 'l_leg_', 5));
r_leg_joints = find(strncmp(joints, 'r_leg_', 5));
n_leg_joints = numel(r_leg_joints);

scale = 0;
options.compl_slack = scale*0.01;
options.lincompl_slack = scale*0.001;
options.jlcompl_slack = scale*0.01;
opt = ContactImplicitFixedPointUnconstrainedProgram(r.getManipulator(), [], options); 


opt = opt.setSolverOptions('snopt','DerivativeOption', 0);
opt = opt.setSolverOptions('snopt','VerifyLevel', 0);
opt = opt.setSolverOptions('snopt','MajorOptimalityTolerance', 1E-5);
opt = opt.setSolverOptions('snopt','SuperbasicsLimit', 1000);
opt = opt.setSolverOptions('snopt','print','snopt.out');

opt = opt.addInputCost(QuadraticSumConstraint(0,0,0.01*eye(nu),zeros(nu,1)));
% remove floating base fredom that we don't need
opt = opt.addConstraint(ConstantConstraint(zeros(3,1)), opt.q_inds([1 2 6]));
opt = opt.addConstraint(BoundingBoxConstraint(-0.1*ones(2,1), 0.1*ones(2,1)), opt.q_inds([4 5]));
opt = opt.addConstraint(BoundingBoxConstraint(0.6, 1.2), opt.q_inds([3]));

% symmetry in the arms
flip = [-1 -1 1 -1 1 -1 1];
A = [eye(n_arm_joints) -diag(flip)];
opt = opt.addConstraint(LinearConstraint(zeros(n_arm_joints,1), zeros(n_arm_joints,1), A), [l_arm_joints;r_arm_joints]);

% symmetry in the legs
flip = [1 -1 1 1 1 -1];
A = [eye(n_leg_joints) -diag(flip)];
opt = opt.addConstraint(LinearConstraint(zeros(n_leg_joints,1), zeros(n_leg_joints,1), A), [l_leg_joints;r_leg_joints]);

% require contact force from feet
%opt = opt.addConstraint(BoundingBoxConstraint(zeros(opt.nC,1)+1, zeros(opt.nC,1)+Inf), opt.l_inds(1:(opt.nD+1):end));
%cp = 8;
%opt = opt.addCost(FunctionHandleConstraint(0, 0, nq, @feet_on_ground_constraint_fun), {opt.q_inds});

% set initial state to fixed point
xstar = 0;
load('/home/gizatt/drc/software/drake/drake/examples/Atlas/data/atlas_fp.mat');
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

end
