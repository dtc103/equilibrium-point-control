#include <iostream>
#include <algorithm>
#include <vector>
#include <unordered_map>
#include <string>
#include <deque>

#include "EQPFinderOpenSim.h"
#include "Tester.h"

Model create_simple_model();

//#define SIMPLEMODEL
#define ARM26
//#define MODEL3D

//makes sure that only one model is active (for testing)
#if defined(SIMPLEMODEL) && defined(ARM26) && defined(MODEL3D)
#undef ARM26
#undef MODEL3D
#elif defined(SIMPLEMODEL) && defined(ARM26) && !defined(MODEL3D)
#undef ARM26
#elif defined(SIMPLEMODEL) && !defined(ARM26) && defined(MODEL3D)
#undef MODEL3D
#elif !defined(SIMPLEMODEL) && defined(ARM26) && defined(MODEL3D)
#undef MODEL3D
#endif


//#define VISUALIZER
//#define DATABASE
//#define CHECK
#ifdef CHECK
//#define LOGGING
#endif

int main() {
#if defined(SIMPLEMODEL)
	Model model = create_simple_model();
#if defined(VISUALIZER)
	model.setUseVisualizer(true);
#endif
	std::vector<double>muscle_activations(0.0, model.getMuscles().getSize());
	VariableConstController* controller = const_cast<VariableConstController*>(dynamic_cast<const VariableConstController*>(&model.getControllerSet().get("SimpleModelControler")));
	controller->setActuators(model.updActuators());

	State& state = model.initSystem();
	model.updMatterSubsystem().setShowDefaultGeometry(false);
	model.equilibrateMuscles(state);
	// Fix the shoulder at its default angle and begin with the elbow flexed.
	model.getJointSet().get("shoulder").getCoordinate().setLocked(state, true);
	//model.getJointSet().get("elbow").getCoordinate().setValue(state, 0.6 * Pi);

	auto jointangles = std::vector<std::tuple<std::string, double>>{ std::make_tuple("elbow", 50) };
	auto activations = std::vector<double>{ 0.5, 0.5 }; //both activations have to be >= 0.1 to get reasonable results

#elif defined(ARM26)
	Model model("./Arm26/arm26.osim");
#if defined(VISUALIZER)
	model.setUseVisualizer(true);
#endif

	std::cout << "--" << std::endl;

	std::vector<double>muscle_activations(model.getMuscles().getSize(), 0.0);
	VariableConstController* controller = new VariableConstController(muscle_activations);

	for (int i = 0; i < model.getMuscles().getSize(); ++i) {
		Thelen2003Muscle* muscle = dynamic_cast<Thelen2003Muscle*>(&model.getMuscles().get(i));
		controller->addActuator(*muscle);
	}

	for (int i = 0; i < model.getMuscles().getSize(); ++i) {
		std::cout << i << ": " << model.getMuscles().get(i).getName() << std::endl;
	}

	for (int i = 0; i < model.getJointSet().getSize(); ++i) {
		std::cout << i << ": " << model.getJointSet().get(i).getName() << std::endl;
	}

	model.addController(controller);
	//first 3 parameters: 
	auto activations = std::vector<double>({ 0.3, 0.2, 0.4, 0.2, 0.2, 0.2 });
	controller->setActivation(activations);

	std::cout << " set activations" << std::endl;
	//std::cout << model.getJointSet().getSize() << std::endl;
#ifdef LOGGING
	ConsoleReporter* reporter = new ConsoleReporter();
	reporter->set_report_time_interval(0.1);
	reporter->addToReport(
		model.getJointSet().get(0).getCoordinate().getOutput("value"),
		"angle_1");
	reporter->addToReport(
		model.getJointSet().get(1).getCoordinate().getOutput("value"),
		"angle_2");
	model.addComponent(reporter);
#endif
	State& state = model.initSystem();

	std::cout << "initialized" << std::endl;

	model.getJointSet().get(1).getCoordinate().setValue(state, 0.1 * Pi);
	model.getJointSet().get(2).getCoordinate().setValue(state, 0.5 * Pi);

	auto jointangles = std::vector<std::tuple<std::string, double>>();
	for (int i = 1; i < model.getJointSet().getSize(); ++i) {
		std::cout << i << std::endl;
		if(i == 1) jointangles.push_back(std::make_tuple(model.getJointSet().get(i).getName(), 45)); //shoulder
		if(i == 2) jointangles.push_back(std::make_tuple(model.getJointSet().get(i).getName(), 135)); //elbow
	}

	//model.getJointSet().get(1).getCoordinate().setLocked(state, true);


#elif defined(MODEL3D)
	Model model("./UpperExtremityModel/MOBL_ARMS_fixed_41.osim");
	
	
#if defined(VISUALIZER)
	model.setUseVisualizer(true);
#endif
	State& state = model.initSystem();
	
	std::cout << model.getJointSet().getSize() << std::endl;

	for (int i = 1; i < model.getJointSet().getSize(); ++i) {
		std::cout << model.getJointSet().get(i).getName() << std::endl;
	}

	model.getJointSet().get("elbow").getCoordinate().setValue(state, 0.2 * Pi);

#ifdef LOGGING
	//// Add a console reporter to print the muscle fiber force and elbow angle.
	ConsoleReporter* reporter = new ConsoleReporter();
	reporter->set_report_time_interval(0.01);
	for (int i = 0; i < model.getJointSet().getSize(); ++i) {
		try {
			reporter->addToReport(model.getJointSet().get(i).get_coordinates(0).getOutput("value"), model.getJointSet().get(i).getName());
		}
		catch (OpenSim::Exception e) {
			std::cout << e.what() << std::endl;
		}
	}
	//    
	model.addComponent(reporter);

	State& state = model.initSystem();

	std::cerr << model.getMuscles().getSize() << std::endl;
#endif
#endif

#if defined(VISUALIZER)
	Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
	viz.setBackgroundType(viz.SolidColor);
	viz.setBackgroundColor(White);
#endif VISUALIZER

#if defined(DATABASE)
	ModelDatabase* db = new ModelDatabase("127.0.0.1", 3306, "root", "Sq4JK&Gb_1998_");
#endif

#ifdef CHECK

	Manager m(model, state);
	state = m.integrate(state.getTime() + 15);

	for (int i = 1; i < model.getJointSet().getSize(); ++i) {
		std::cout << i << ": ";
		std::cout << convertRadiansToDegrees(model.getJointSet().get(i).getCoordinate().getValue(state)) << " ";
	}
	std::cout << std::endl;

	controller->setActivation(std::vector<double>({ 0, 0.1, 0.5, 0.2, 0.2, 0.5 }));

	state = m.integrate(state.getTime() + 15);

	for (int i = 1; i < model.getJointSet().getSize(); ++i) {
		std::cout << convertRadiansToDegrees(model.getJointSet().get(i).getCoordinate().getValue(state)) << " ";
	}
	std::cout << std::endl;


	std::cout << "came until here" << std::endl;
	//simulate(model, state, 30);

#endif CHECK

#ifndef CHECK

	/////////// WRITE CODE HERE ////////////////

	std::vector<double>({ 0.3, 0.2, 0.4, 0.2, 0.2, 0.1 }); //-53.6708 2: 125.278
	std::vector<double>({ 0, 0.1, 0.5, 0.2, 0.2, 0.4 }); //44.8168 135.223

	//EQPFinderOpenSim eqpfinder(model, state, *controller);
	//eqpfinder.set_angle_error_delta(0.01);
	//eqpfinder.pattern_find(jointangles, activations, 0.001);
	//eqpfinder.find(jointangles, activations);

	Tester t(model, state);
	//t.goal_angle_pattern_search_simple_model_random(activations, jointangles, 10);
	//t.goal_angle_brute_force_simple_model_deterministic(activations, jointangles, 10);
	//t.goal_angle_brute_force_simple_model_random(activations, jointangles, 10);
	//t.test_stepsize_brute_force(activations, jointangles, 0.001, 0.056, 0.005);
	t.test_stepsize_pattern_search(activations, jointangles, 0.001, 0.056, 0.005);
	

	/*
	eqpfinder.set_angle_error_delta(0.01);
	eqpfinder.set_gradient_stepsize(0.001);

	std::cout << "start find method" << std::endl;
	
	auto res = eqpfinder.find(jointangles, activations);

	for (auto angle : std::get<1>(res)) {
		std::cout << angle << " ";
	}
	std::cout << std::endl;

	for (auto activation : std::get<0>(res)) {
		std::cout << activation << " ";
	}
	std::cout << std::endl;
	*/
	/*
	eqpfinder.set_gradient_stepsize(0.001);
	eqpfinder.set_angle_error_delta(0.001);
	auto f = eqpfinder.find(std::vector < std::tuple<std::string, double>>{std::make_tuple("shoulder", 50.0), std::make_tuple("elbow", 50.0)}, std::vector<double>{0.5, 0.5});
	std::cout << "Activations: ";
	for (int i = 0; i < std::get<0>(f).size(); ++i) {
		std::cout << std::get<0>(f)[i] << " ";
	}
	std::cout << "\n";

	std::cout << "Angles: ";
	for (int i = 0; i < std::get<1>(f).size(); ++i) {
		std::cout << std::get<1>(f)[i] << " ";
	}
	std::cout << "\n";
	*/
#endif
	return 0;

};


Model create_simple_model() {
	Model model;
	model.setName("biceps_curl");

	// Create two links, each with a mass of 1 kg, center of mass at the body's
	// origin, and moments and products of inertia of zero.
	OpenSim::Body* humerus = new OpenSim::Body("humerus", 1, Vec3(0), Inertia(0));
	OpenSim::Body* radius = new OpenSim::Body("radius", 1, Vec3(0), Inertia(0));
	// Connect the bodies with pin joints. Assume each body is 1m long.
	PinJoint* shoulder = new PinJoint("shoulder",
		// Parent body, location in parent, orientation in parent.
		model.getGround(), Vec3(0), Vec3(0),
		// Child body, location in child, orientation in child.
		*humerus, Vec3(0, 0.26, 0), Vec3(0));
	PinJoint* elbow = new PinJoint("elbow",
		*humerus, Vec3(0), Vec3(0),
		*radius, Vec3(0, 0.26, 0), Vec3(0));

	// Add a muscle that flexes the elbow.
	Millard2012EquilibriumMuscle* biceps = new Millard2012EquilibriumMuscle("biceps", 200, 0.20, 0.14, 0);
	biceps->addNewPathPoint("origin", *humerus, Vec3(0 + 0.03, 0.26 + 0.03, 0));
	biceps->addNewPathPoint("insertion", *radius, Vec3(0 + 0.03, 0.26 - 0.05, 0));
	Millard2012EquilibriumMuscle* triceps = new Millard2012EquilibriumMuscle("triceps", 0.8 * 200, 0.20, 0.118, 0);
	triceps->addNewPathPoint("origin", *humerus, Vec3(-0.05, 0.26, 0));
	triceps->addNewPathPoint("insertion", *radius, Vec3(-0.05, 0.26 + 0.02, 0));

	VariableConstController* brain = new VariableConstController(std::vector<double>{0.5, 0.5});
	brain->setName("SimpleModelControler");
	brain->addActuator(*biceps);
	brain->addActuator(*triceps);

	// Add components to the model.
	model.addBody(humerus);    model.addBody(radius);
	model.addJoint(shoulder);  model.addJoint(elbow);
	model.addForce(biceps);
	model.addForce(triceps);
	model.addController(brain);


#ifdef LOGGING
	//// Add a console reporter to print the muscle fiber force and elbow angle.
	ConsoleReporter* reporter = new ConsoleReporter();
	reporter->set_report_time_interval(0.01);
	reporter->addToReport(biceps->getOutput("fiber_force"), "biceps_fibre_force");
	reporter->addToReport(
		elbow->getCoordinate(PinJoint::Coord::RotationZ).getOutput("value"),
		"elbow_angle");
	reporter->addToReport(elbow->getCoordinate(PinJoint::Coord::RotationZ).getOutput("acceleration"), "elbow_acceleration");
	reporter->addToReport(
		shoulder->getCoordinate(PinJoint::Coord::RotationZ).getOutput("value"),
		"shoulder_angle");
	//    
	model.addComponent(reporter);
#endif

	// Add display geometry.
	Ellipsoid bodyGeometry(0.02, 0.13, 0.02);
	bodyGeometry.setColor(Gray);
	// Attach an ellipsoid to a frame located at the center of each body.
	PhysicalOffsetFrame* humerusCenter = new PhysicalOffsetFrame(
		"humerusCenter", *humerus, Transform(Vec3(0, 0.13, 0)));
	humerus->addComponent(humerusCenter);
	humerusCenter->attachGeometry(bodyGeometry.clone());
	PhysicalOffsetFrame* radiusCenter = new PhysicalOffsetFrame(
		"radiusCenter", *radius, Transform(Vec3(0, 0.13, 0)));
	radius->addComponent(radiusCenter);
	radiusCenter->attachGeometry(bodyGeometry.clone());

	// Configure the model.


	return model;
}

//ModelDatabase db("127.0.0.1", 3306, "root", "Sq4JK&Gb_1998_");

/*
speed
value
acceleration
*/

Model create_arm26_model() {
	Model model;
	model.setName("arm26");

	// Create two links, each with a mass of 1 kg, center of mass at the body's
	// origin, and moments and products of inertia of zero.
	OpenSim::Body* humerus = new OpenSim::Body("humerus", 1, Vec3(0), Inertia(0));
	OpenSim::Body* radius = new OpenSim::Body("radius", 1, Vec3(0), Inertia(0));
	// Connect the bodies with pin joints. Assume each body is 1m long.
	PinJoint* shoulder = new PinJoint("shoulder",
		// Parent body, location in parent, orientation in parent.
		model.getGround(), Vec3(0), Vec3(0),
		// Child body, location in child, orientation in child.
		*humerus, Vec3(0, 0.26, 0), Vec3(0));
	PinJoint* elbow = new PinJoint("elbow",
		*humerus, Vec3(0), Vec3(0),
		*radius, Vec3(0, 0.26, 0), Vec3(0));

	// Add a muscle that flexes the elbow.
	Millard2012EquilibriumMuscle* biceps = new Millard2012EquilibriumMuscle("biceps", 200, 0.20, 0.14, 0);
	biceps->addNewPathPoint("origin", *humerus, Vec3(0 + 0.03, 0.26 + 0.03, 0));
	biceps->addNewPathPoint("insertion", *radius, Vec3(0 + 0.03, 0.26 - 0.05, 0));
	Millard2012EquilibriumMuscle* triceps = new Millard2012EquilibriumMuscle("triceps", 0.8 * 200, 0.20, 0.118, 0);
	triceps->addNewPathPoint("origin", *humerus, Vec3(-0.05, 0.26, 0));
	triceps->addNewPathPoint("insertion", *radius, Vec3(-0.05, 0.26 + 0.02, 0));

	VariableConstController* brain = new VariableConstController(std::vector<double>{0.486, 0.507});
	brain->setName("SimpleModelControler");
	brain->addActuator(*biceps);
	brain->addActuator(*triceps);

	// Add components to the model.
	model.addBody(humerus);    model.addBody(radius);
	model.addJoint(shoulder);  model.addJoint(elbow);
	model.addForce(biceps);
	model.addForce(triceps);
	model.addController(brain);


#ifdef LOGGING
	//// Add a console reporter to print the muscle fiber force and elbow angle.
	ConsoleReporter* reporter = new ConsoleReporter();
	reporter->set_report_time_interval(0.01);
	reporter->addToReport(biceps->getOutput("fiber_force"), "biceps_fibre_force");
	reporter->addToReport(
		elbow->getCoordinate(PinJoint::Coord::RotationZ).getOutput("value"),
		"elbow_angle");
	reporter->addToReport(elbow->getCoordinate(PinJoint::Coord::RotationZ).getOutput("acceleration"), "elbow_acceleration");
	reporter->addToReport(
		shoulder->getCoordinate(PinJoint::Coord::RotationZ).getOutput("value"),
		"shoulder_angle");
	//    
	model.addComponent(reporter);
#endif

	// Add display geometry.
	Ellipsoid bodyGeometry(0.02, 0.13, 0.02);
	bodyGeometry.setColor(Gray);
	// Attach an ellipsoid to a frame located at the center of each body.
	PhysicalOffsetFrame* humerusCenter = new PhysicalOffsetFrame(
		"humerusCenter", *humerus, Transform(Vec3(0, 0.13, 0)));
	humerus->addComponent(humerusCenter);
	humerusCenter->attachGeometry(bodyGeometry.clone());
	PhysicalOffsetFrame* radiusCenter = new PhysicalOffsetFrame(
		"radiusCenter", *radius, Transform(Vec3(0, 0.13, 0)));
	radius->addComponent(radiusCenter);
	radiusCenter->attachGeometry(bodyGeometry.clone());

	// Configure the model.


	return model;
}