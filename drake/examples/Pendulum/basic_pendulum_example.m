dt = .002;

r = TimeSteppingRigidBodyManipulator('Pendulum.urdf', dt);
r = setSimulinkParam(r,'MinStep','0.001');

% Add obstacle
world = r.findLinkId('world');
box = RigidBodyBox([.1 .1 .1], [-0.3 0 -0.3], [0 0 0]);
box = box.setColor([0 0 1]);
r = addGeometryToBody(r, world, box);
r = r.compile();


% Construct visualiser and controller
v = r.constructVisualizer;
c = PDController(r);
sys = feedback(r, c);

x0 = [r.getManipulator().getRandomConfiguration; zeros(r.getManipulator().num_velocities,1)];

xtraj = simulate(sys, [0 5], x0);
v.playback(xtraj)

