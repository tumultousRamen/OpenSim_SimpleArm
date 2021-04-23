import org.opensim.modeling.*
arm = Model();
arm.setUseVisualizer(true);

%getting ground reference 
ground = arm.getGround();

%setting constants
torsoWidth = 0.5;
limbLength = 1;
bodyGeometry = Ellipsoid(limbLength/10, limbLength/2, limbLength/10);
bodyGeometry.setColor(Vec3(0.6));

%Creating a torso
torso = Body();
torso.setName('Torso');
torso.setMass(30);
torso.setInertia( Inertia(1,1,1,0,0,0));
torso.attachGeometry(Sphere(torsoWidth));
arm.addBody(torso);

%Creating a Pin Joint for the Torso Body
locationInParent    = Vec3(2,2,2);
orientationInParent = Vec3(0,0,0);
locationInChild     = Vec3(0,0,0);
orientationInChild  = Vec3(0,0,0);
torsoToGround = PinJoint('TorsoToGround', ground, locationInParent,...
    orientationInParent, torso, locationInChild, orientationInChild);

%Updating the coordinates of the torso to ground joint 
torso_rz = torsoToGround.upd_coordinates(0);
torso_rz.setRange([deg2rad(-100), deg2rad(100)]);
torso_rz.setName('torso_rz');
torso_rz.setDefaultValue(deg2rad(0));
torso_rz.setDefaultSpeedValue(0);
torso_rz.setDefaultLocked(true);

%Adding Joint to the model
arm.addJoint(torsoToGround);

%Creating a humerus 
humerus = Body();
humerus.setName('Humerus');
humerus.setMass(1);
humerus.setMassCenter( Vec3(0,0,0) );
humerus.setInertia( Inertia(0) );
humerus.attachGeometry(bodyGeometry.clone());
arm.addBody(humerus);

%Creating a Pin Joint for the shoulder
locationInParent = Vec3(0,0,torsoWidth);
orientationInParent = Vec3(0,0,0);
locationInChild = Vec3(0,limbLength/2,0);
orientationInChild = Vec3(0,0,0);
Shoulder = PinJoint('Shoulder', torso, locationInParent, orientationInParent,...
    humerus, locationInChild, orientationInChild);

%Updating the co-ordinates of the shoulder joint
Shoulder_rz = Shoulder.upd_coordinates(0);
Shoulder_rz.setRange([deg2rad(-100), deg2rad(100)]);
Shoulder_rz.setName('Shoulder_rz');
Shoulder_rz.setDefaultValue(deg2rad(30));
Shoulder_rz.setDefaultSpeedValue(0 );
%Add Joint to the model
arm.addJoint(Shoulder);

%Creating a radius
radius = Body();
radius.setName('radius');
radius.setMass(1);
radius.setMassCenter( Vec3(0,0,0) );
radius.setInertia( Inertia(0) );
radius.attachGeometry(bodyGeometry.clone());
arm.addBody(radius);

%Creating a Pin Joint for the elbow
locationInParent = Vec3(0,-limbLength/2,0);
orientationInParent = Vec3(0,0,0);
locationInChild = Vec3(0,limbLength/2,0);
orientationInChild = Vec3(0,0,0);
Elbow = PinJoint('elbow', humerus, locationInParent, orientationInParent,...
    radius, locationInChild, orientationInChild);

%Updating the co-ordinates of the elbow joint
Elbow_rz = Shoulder.upd_coordinates(0);
Elbow_rz.setRange([deg2rad(-45), deg2rad(0)]);
Elbow_rz.setName('Elbow_rz');
Elbow_rz.setDefaultValue(deg2rad(0));
Elbow_rz.setDefaultSpeedValue(0 );
%Add Joint to the model
arm.addJoint(Elbow);

%Creating and adding biceps origin and insertion points
biceps = Millard2012EquilibriumMuscle('biceps', 200, 0.6, 0.55, 0);
biceps.addNewPathPoint('origin', humerus, Vec3(0, - limbLength/2 + 0.8, 0));
biceps.addNewPathPoint('insertion', radius,  Vec3(0, - limbLength/2 + 0.7, 0));
%Adding biceps to the model
arm.addForce(biceps);

%Creating and adding an actuator and controller
brain = PrescribedController();
brain.addActuator(biceps);
customActuation = StepFunction(0.5, 3, 0.3, 1);
brain.prescribeControlForActuator('biceps', customActuation );
arm.addController(brain);

%Contact Sphere
contactRadius = 0.1;

%Creating a Shoulder Contact Sphere
ShlCntSphere = ContactSphere();
ShlCntSphere.setRadius(contactRadius);
ShlCntSphere.setFrame(torso);
ShlCntSphere.setLocation(Vec3(0,0, torsoWidth));
ShlCntSphere.setName('ShoulderContact');
arm.addContactGeometry(ShlCntSphere);

%Creating Elbow Contact Sphere
EbwCntSphere = ContactSphere();
EbwCntSphere.setRadius(contactRadius);
EbwCntSphere.setFrame(radius);
EbwCntSphere.setLocation(Vec3(0,limbLength/2,0));
EbwCntSphere.setName('ElbowContact');
arm.addContactGeometry(EbwCntSphere);

%Creating a reporter to gauge and report the simulation output
reporter = ConsoleReporter();
reporter.set_report_time_interval(1.0);
reporter.addToReport(biceps.getOutput('fiber_force'));
reporter.addToReport(Elbow.getCoordinate().getOutput('value'), 'Elbow_angle');
arm.addComponent(reporter);

%Arm is configured and elements set at default angles.
state = arm.initSystem();
Shoulder.getCoordinate().setLocked(state, true);
Elbow.getCoordinate().setValue(state, 0.5 * pi);
arm.equilibrateMuscles(state);

%Arm is simulated 
manager = Manager(arm);
finalTime = 10;
manager.initialize( state );
manager.integrate( finalTime );


