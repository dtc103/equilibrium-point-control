#pragma once

#include <fstream>
#include <set>
#include <random>
#include <algorithm>

#include "EQPFinderOpenSim.h"


class Tester {
public:
	Tester(Model& model, State& state) : model(model), state(state) {

	}

	void goal_angle_brute_force_simple_model_deterministic(std::vector<double>start_activations, std::vector<std::tuple<std::string, double>>jointangles, int runs) {
		std::ofstream runtime_file = get_csv_header("goal_angle_brute_force_simple_model_deterministic");

		VariableConstController* controller = const_cast<VariableConstController*>(dynamic_cast<const VariableConstController*>(&model.getControllerSet().get(0)));
		controller->setActivation(std::vector<double>(start_activations));

		EQPFinderOpenSim eqpfinder(this->model, this->state, *controller);
		
		eqpfinder.set_gradient_stepsize(0.001);
		eqpfinder.set_angle_error_delta(0.01);
		for (int i = 0; i < runs; ++i) {
			try {
				controller->setActivation(std::vector<double>(start_activations));

				std::cout << "run: " << i << "\n";
				auto start = std::chrono::system_clock::now();
				std::tuple<std::vector<double>, std::vector<double>> results = eqpfinder.find(jointangles, start_activations);
				auto end = std::chrono::system_clock::now();

				for (int i = 0; i < std::get<0>(results).size(); ++i) {
					runtime_file << start_activations[i] << ";"; //start_activation
					runtime_file << std::get<0>(results)[i] << ";"; //final_activation
				}

				for (int i = 0; i < std::get<1>(results).size(); ++i) {
					runtime_file << std::get<1>(jointangles[i]) << ";"; //goal_angle
					runtime_file << std::get<1>(results)[i] << ";"; //actual_angle
				}

				runtime_file << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << ";";
				runtime_file << eqpfinder.get_gradient_stepsize() << std::endl;
			}
			catch (OpenSim::Exception e) {
				runtime_file << e.what() << std::endl;
			}
		}
	}

	void goal_angle_brute_force_simple_model_random(std::vector<double>start_activations, std::vector<std::tuple<std::string, double>>jointangles, int runs) {
		std::ofstream runtime_file = get_csv_header("goal_angle_brute_force_simple_model_random");

		VariableConstController* controller = const_cast<VariableConstController*>(dynamic_cast<const VariableConstController*>(&model.getControllerSet().get(0)));
		controller->setActivation(std::vector<double>(start_activations));

		EQPFinderOpenSim eqpfinder(this->model, this->state, *controller);
		eqpfinder.set_random_threshold(0);

		eqpfinder.set_gradient_stepsize(0.001);
		eqpfinder.set_angle_error_delta(0.01);
		for (int i = 0; i < runs; ++i) {
			try {
				controller->setActivation(std::vector<double>(start_activations));

				std::cout << "run: " << i << "\n";
				auto start = std::chrono::system_clock::now();
				std::tuple<std::vector<double>, std::vector<double>> results = eqpfinder.find(jointangles, start_activations);
				auto end = std::chrono::system_clock::now();

				for (int i = 0; i < std::get<0>(results).size(); ++i) {
					runtime_file << start_activations[i] << ";"; //start_activation
					runtime_file << std::get<0>(results)[i] << ";"; //final_activation
				}

				for (int i = 0; i < std::get<1>(results).size(); ++i) {
					runtime_file << std::get<1>(jointangles[i]) << ";"; //goal_angle
					runtime_file << std::get<1>(results)[i] << ";"; //actual_angle
				}

				runtime_file << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << ";";
				runtime_file << eqpfinder.get_gradient_stepsize() << std::endl;
			}
			catch (OpenSim::Exception e) {
				runtime_file << e.what() << std::endl;
			}
		}
	}

	void goal_angle_pattern_search_simple_model_random(std::vector<double>start_activations, std::vector<std::tuple<std::string, double>>jointangles, int runs) {
		std::ofstream runtime_file = get_csv_header("goal_angle_pattern_search_simple_model");

		VariableConstController* controller = const_cast<VariableConstController*>(dynamic_cast<const VariableConstController*>(&model.getControllerSet().get(0)));
		for (int i = 0; i < model.getMuscles().getSize(); ++i) {
			controller->setActivationsAt(i, start_activations[i]);
		}

		EQPFinderOpenSim eqpfinder(this->model, this->state, *controller);

		eqpfinder.set_gradient_stepsize(0.001);
		eqpfinder.set_angle_error_delta(0.01);
		for (int i = 0; i < runs; ++i) {
			try {
				controller->setActivation(std::vector<double>(start_activations));
				std::cout << "run: " << i << "\n";
				auto start = std::chrono::system_clock::now();
				std::tuple<std::vector<double>, std::vector<double>> results = eqpfinder.pattern_find(jointangles, start_activations, 0.001);
				auto end = std::chrono::system_clock::now();

				for (int i = 0; i < std::get<0>(results).size(); ++i) {
					runtime_file << start_activations[i] << ";"; //start_activation
					runtime_file << std::get<0>(results)[i] << ";"; //final_activation
				}

				for (int i = 0; i < std::get<1>(results).size(); ++i) {
					runtime_file << std::get<1>(jointangles[i]) << ";"; //goal_angle
					runtime_file << std::get<1>(results)[i] << ";"; //actual_angle
				}

				runtime_file << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << ";";
				runtime_file << eqpfinder.get_gradient_stepsize() << std::endl;
			}
			catch (OpenSim::Exception e) {
				runtime_file << e.what() << std::endl;
			}
		}
	}

	
	void test_stepsize_brute_force(std::vector<double>start_activations, std::vector<std::tuple<std::string, double>>jointangles, double from, double to, double step_size){
		std::ofstream integration_steptime_file = get_csv_header("step_size_brute_force");

		VariableConstController* controller = const_cast<VariableConstController*>(dynamic_cast<const VariableConstController*>(&model.getControllerSet().get(0)));
		for (int i = 0; i < model.getMuscles().getSize(); ++i) {
			controller->setActivationsAt(i, start_activations[i]);
		}
		
		EQPFinderOpenSim eqpfinder(this->model, this->state, *controller);
		for (double i = from; i <= to; i += step_size) {
			try {
				eqpfinder.set_gradient_stepsize(i);

				auto start = std::chrono::system_clock::now();
				std::tuple<std::vector<double>, std::vector<double>> results = eqpfinder.find(jointangles, start_activations);
				auto end = std::chrono::system_clock::now();

				for (int i = 0; i < std::get<0>(results).size(); ++i) {
					integration_steptime_file << start_activations[i] << ";";
					integration_steptime_file << std::get<0>(results)[i] << ";";
				}

				for (int i = 0; i < std::get<1>(results).size(); ++i) {
					integration_steptime_file << std::get<1>(jointangles[i]) << ";";
					integration_steptime_file << std::get<1>(results)[i] << ";";
				}

				integration_steptime_file << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << ";";
				integration_steptime_file << i << ";";
				integration_steptime_file << eqpfinder.get_angle_error_delta() << ";";
				integration_steptime_file << eqpfinder.get_gradient_stepsize() << std::endl;
			}
			catch (OpenSim::Exception e) {
				integration_steptime_file << e.what() << std::endl;
			}
		}
	}

	void test_stepsize_pattern_search(std::vector<double>start_activations, std::vector<std::tuple<std::string, double>>jointangles, double from, double to, double step_size) {
		std::ofstream integration_steptime_file = get_csv_header("step_size_pattern_search");

		VariableConstController* controller = const_cast<VariableConstController*>(dynamic_cast<const VariableConstController*>(&model.getControllerSet().get(0)));
		for (int i = 0; i < model.getMuscles().getSize(); ++i) {
			controller->setActivationsAt(i, start_activations[i]);
		}

		EQPFinderOpenSim eqpfinder(this->model, this->state, *controller);
		for (double i = from; i <= to; i += step_size) {
			try {

				auto start = std::chrono::system_clock::now();
				std::tuple<std::vector<double>, std::vector<double>> results = eqpfinder.pattern_find(jointangles, start_activations, i);
				auto end = std::chrono::system_clock::now();

				for (int k = 0; k < std::get<0>(results).size(); ++k) {
					integration_steptime_file << start_activations[k] << ";";
					integration_steptime_file << std::get<0>(results)[k] << ";";
				}

				for (int k = 0; k < std::get<1>(results).size(); ++k) {
					integration_steptime_file << std::get<1>(jointangles[k]) << ";";
					integration_steptime_file << std::get<1>(results)[k] << ";";
				}

				integration_steptime_file << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << ";";
				integration_steptime_file << eqpfinder.get_gradient_stepsize() << std::endl;
			}
			catch (OpenSim::Exception e) {
				integration_steptime_file << e.what() << std::endl;
			}
		}
	}

	void goal_angle_distance_brute_force_simple_model(std::vector<double>start_activations, std::vector<std::tuple<std::string, double>>jointangles) {
		std::ofstream runtime_file = get_csv_header("goal_angle_distance_brute_force_simple_model");

		VariableConstController* controller = const_cast<VariableConstController*>(dynamic_cast<const VariableConstController*>(&model.getControllerSet().get(0)));
		for (int i = 0; i < model.getMuscles().getSize(); ++i) {
			controller->setActivationsAt(i, start_activations[i]);
		}

		EQPFinderOpenSim eqpfinder(this->model, this->state, *controller);
		eqpfinder.set_gradient_stepsize(0.001);
		eqpfinder.set_angle_error_delta(0.01);
		for (int i = 10; i <= 80; i+=5) {
			try {
				std::cout << "run: " << i << "\n";
				auto start = std::chrono::system_clock::now();
				std::tuple<std::vector<double>, std::vector<double>> results = eqpfinder.find(std::vector<std::tuple<std::string, double>>{std::make_tuple("elbow", i)}, start_activations);
				auto end = std::chrono::system_clock::now();

				for (int k = 0; k < std::get<0>(results).size(); ++k) {
					runtime_file << start_activations[k] << ";"; //start_activation
					runtime_file << std::get<0>(results)[k] << ";"; //final_activation
				}

				for (int k = 0; k < std::get<1>(results).size(); ++k) {
					runtime_file << i << ";"; //goal_angle
					runtime_file << std::get<1>(results)[k] << ";"; //actual_angle
				}

				runtime_file << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << ";";
				runtime_file << eqpfinder.get_gradient_stepsize() << std::endl;
			}
			catch (OpenSim::Exception e) {
				runtime_file << e.what() << std::endl;
			}
		}
	}

	void goal_angle_distance_pattern_search_simple_model(std::vector<double>start_activations, std::vector<std::tuple<std::string, double>>jointangles) {
		std::ofstream runtime_file = get_csv_header("goal_angle_distance_pattern_search_simple_model");

		VariableConstController* controller = const_cast<VariableConstController*>(dynamic_cast<const VariableConstController*>(&model.getControllerSet().get(0)));
		for (int i = 0; i < model.getMuscles().getSize(); ++i) {
			controller->setActivationsAt(i, start_activations[i]);
		}

		EQPFinderOpenSim eqpfinder(this->model, this->state, *controller);

		eqpfinder.set_gradient_stepsize(0.001);
		eqpfinder.set_angle_error_delta(0.01);
		for (int i = 10; i <= 80; i += 5) {
			try {
				std::cout << "run: " << i << "\n";
				auto start = std::chrono::system_clock::now();
				std::tuple<std::vector<double>, std::vector<double>> results = eqpfinder.pattern_find(std::vector<std::tuple<std::string, double>>{std::make_tuple("elbow", i)}, start_activations, 0.001);
				auto end = std::chrono::system_clock::now();

				for (int k= 0; k < std::get<0>(results).size(); ++k) {
					runtime_file << start_activations[k] << ";"; //start_activation
					runtime_file << std::get<0>(results)[k] << ";"; //final_activation
				}

				for (int k = 0; k < std::get<1>(results).size(); ++k) {
					runtime_file << i << ";"; //goal_angle
					runtime_file << std::get<1>(results)[k] << ";"; //actual_angle
				}

				runtime_file << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << ";";
				runtime_file << eqpfinder.get_gradient_stepsize() << std::endl;
			}
			catch (OpenSim::Exception e) {
				runtime_file << e.what() << std::endl;
			}
		}
	}

	std::ofstream get_csv_header(std::string filename, std::vector<std::string>additional_fields = std::vector<std::string>{}) {
		std::ofstream csv_file(testfolder_path + filename + "_" + model.getName() + ".csv", std::ofstream::out);

		for (int i = 0; i < model.getMuscles().getSize(); ++i) {
			csv_file << model.getMuscles().get(i).getName() << "_start_activation;";
			csv_file << model.getMuscles().get(i).getName() << "_final_activation;";
		}

		for (int i = 1; i < model.getJointSet().getSize(); ++i) {
			csv_file << model.getJointSet().get(i).getName() << "_jointangle_desired;";
			csv_file << model.getJointSet().get(i).getName() << "_jointangle_final;";
		}
		csv_file << "execution_time;";
		csv_file << "gradient_stepsize";

		for (auto additionals : additional_fields) {
			csv_file << (";" + additionals);
		}

		csv_file << std::endl;

		return csv_file;
	}

	std::vector<double> random_vector(int size, double min, double max) {
		std::random_device rnd_device;
		std::mt19937 mersenne_engine{ rnd_device() };
		std::uniform_real_distribution<double> dist{ min, max };

		auto gen = [&dist, &mersenne_engine]() {
			return std::round(dist(mersenne_engine) * 1000) / 1000;
		};

		std::vector<double> vec(size);
		std::generate(vec.begin(), vec.end(), gen);

		return vec;
	}

private:
	std::string testfolder_path = "./tests/";
	Model& model;
	State& state;
};