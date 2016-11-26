#include "ActuatorDynamics.h"
#include <iostream>

using namespace ActuatorDynamicsTools;

int main(int argc, char** argv) {
	int order = 1;
	double u_max = 1;
	double t = 1;

	ActuatorDynamics actuator = ActuatorDynamics(order, u_max);

	std::cout << "instantiated the actuator dynamics object " << std::endl;
	// run a few tests here . . .
	std::vector<double> bounds = actuator.getBounds(t);

	std::cout << "initial bounds are " << bounds[0] << ' ' << bounds[1] << std::endl;


	std::cout << "processing sample for actuator \n";
	actuator.processSample(t, 0.5);

	bounds = actuator.getBounds(t + 1);
	std::cout << "bounds after update are " << bounds[0] << ' ' << bounds[1] << std::endl;
}

