#pragma once
#include <OpenSim/OpenSim.h>
#include <OpenSim/Actuators/ModelOperators.h>
#include <OpenSim/Moco/osimMoco.h>
#include <vector>
#include <string>
#include <iostream>
#include <random>
#include <algorithm>
#include <numeric>
#include <thread>

#include "model_database.h"

using namespace SimTK;
using namespace OpenSim;

class VariableConstController : public Controller {
    OpenSim_DECLARE_CONCRETE_OBJECT(VariableConstController, Controller);
public:
    VariableConstController(std::vector<double> initial_activations) : Controller(), activations(initial_activations) {}

    void computeControls(const SimTK::State& s, SimTK::Vector& controls) const {
        if (getActuatorSet().getSize() != this->activations.size()) {
            throw "Size of muscles does not fit to activations";
        }

        for (int i = 0; i < getActuatorSet().getSize(); ++i) {
            Vector control(1, this->activations[i]);
            getActuatorSet().get(i).addInControls(control, controls);
            //std::cout << typeid(getActuatorSet().get(i)).name() << std::endl;
        }
    }

    std::vector<double>& getActivations(){
        return this->activations;
    }

    int getMuscleAmount() const{
        return getActuatorSet().getSize();
    }

    void setActivation(std::vector<double> new_activations) {
        if (getActuatorSet().getSize() != new_activations.size()) {
            throw "Size of muscles does not fit to activations";
        }
        this->activations = new_activations;
    }

    void setActivationsAt(int muscle_index, double activation) {
        if (muscle_index >= this->activations.size()) {
            return;
        }
        else {
            this->activations[muscle_index] = activation;
        }
    }

private:
    std::vector<double> activations;
};


class EQPFinderOpenSim {
public:
    EQPFinderOpenSim(Model& model, State& state, VariableConstController& controller) : model(model), initial_state(state), controller(controller) {}
    EQPFinderOpenSim(Model& model, State& state, VariableConstController& controller, ModelDatabase* db) : model(model), initial_state(state), controller(controller), db(db) {
        std::vector<std::string> muscle_names;

        for (int i = 0; i < model.getMuscles().getSize(); ++i) {
            std::string name = model.getMuscles().get(i).getName();
            muscle_names.push_back(name);
        }
        
        this->db->create_table(model.getName(), muscle_names);
    }
    
    

    //returns the name and corresponding muscle activity for every muscle
    std::tuple<std::vector<double>, std::vector<double>> find(std::vector<std::tuple<std::string, double>> jointangles, std::vector<double> initial_muscle_activities) {
        
        //random part initialization
        //how often we have to get the same result to be certain that this result is our best one
        //std::deque<std::vector<double>> certainity_values(this->error_certainity - 2, std::vector<double>(jointangles.size(), std::numeric_limits<double>::max()));
        //certainity_values.push_back(std::vector<double>(jointangles.size(), 0));
        //certainity_values.push_back(std::vector<double>(jointangles.size(), 0));

        bool random_approach = false;
        if (this->controller.getMuscleAmount() >= this->random_threshold) {
            random_approach = true;
            std::cout << "random approach" << std::endl;
        }
        
        std::vector<double> final_muscle_activations;
        std::vector<double> best_intermediate_result = initial_muscle_activities;

        State best_state(this->initial_state);
        std::vector<double> min_differences(jointangles.size(), std::numeric_limits<double>::max());
        std::vector<double> best_angles;

        std::set<std::vector<double>> visited;

        while(!random_approach /* || !this->is_certain(certainity_values) */) {
            bool has_changed = false;
            std::vector<std::vector<int>> permutations = calc_combinations(controller.getMuscleAmount());

            for (auto permutation : permutations) {
                std::vector<double> muscle_activities = best_intermediate_result;

                for (int i = 0; i < permutation.size(); ++i) {
                    muscle_activities[i] += (double)permutation[i] * this->gradient_stepsize;
                    if (muscle_activities[i] < 0.0) {
                        muscle_activities[i] = 0.0;
                    }
                    if (muscle_activities[i] > 1.0) {
                        muscle_activities[i] = 1.0;
                    }
                }
                
                /*std::cout << "muscles: ";
                for (auto el : muscle_activities) {
                    std::cout << el << " ";
                }
                std::cout << std::endl;*/
                
                
                //if (visited.find(muscle_activities) != visited.end()) {
                //    //std::cout << "--> denied" << std::endl;
                //    continue;
                //}
                
                //std::cout << std::endl;
               
                controller.setActivation(muscle_activities);

                //std::cout << "start simulation" << std::endl;

                State& intermed_state = this->simulate_until_stop(best_state);

                //std::cout << "simulation ended" << std::endl;

                std::vector<double> current_angles = get_current_angles(intermed_state, jointangles);
                std::vector<double> current_differences = get_current_differences(intermed_state, jointangles, current_angles);
                
                //std::cout << "\n";
                
                if (total_error_got_better(current_differences, min_differences)) {

                    std::cout << "-----------> New minimal difference: " << std::endl;
                    for (int i = 0; i < jointangles.size(); ++i) {
                        std::cout << convertRadiansToDegrees( model.getJointSet().get(std::get<0>(jointangles[i])).getCoordinate().getValue(intermed_state) )<< " ";
                    }
                    std::cout << std::endl;
                    
                    best_state = intermed_state;
                    min_differences = current_differences;
                    best_intermediate_result = muscle_activities;
                    best_angles = current_angles;
                    has_changed = true;
                }
                //visited.insert(muscle_activities);
            }

            if (random_approach) {
                //certainity_values.pop_front();
                //certainity_values.push_back(min_differences);
            }

            if (!random_approach) {
                if (!has_changed) {
                    //std::cout << "got called" << std::endl;
                    break;
                }
            }
            final_muscle_activations = best_intermediate_result;
        }

        if (this->db != nullptr) {
            //std::vector<std::tuple<std::string, double>> muscle_name_activations = create_muscle_name_activation_list(model.getMuscles(), final_muscle_activation);
            //this->db->safe_data(model.getName(), final_angle, muscle_name_activations);
        }

        return std::make_tuple(final_muscle_activations, best_angles);
    }
 

    std::tuple<std::vector<double>, std::vector<double>> pattern_find(std::vector<std::tuple<std::string, double>> jointangles, std::vector<double> initial_muscle_activities, double accuracy) {
        std::vector<double> final_muscle_activations;
        std::vector<double> best_intermediate_result = initial_muscle_activities;
        State best_state(this->initial_state);

        std::vector<double> min_differences(jointangles.size(), std::numeric_limits<double>::max());
        std::vector<double> best_angles;
        
        controller.setActivation(best_intermediate_result);
        State& intermed_state = simulate_until_stop(best_state);

        std::vector<double> current_angles = get_current_angles(intermed_state, jointangles);
        std::vector<double> current_differences = get_current_differences(intermed_state, jointangles, current_angles);
        
        if (total_error_got_better(current_differences, min_differences)) {
            best_state = intermed_state;
            min_differences = current_differences;
            best_angles = current_angles;
        }

        double step_size = 0.1;
        bool goal = false;

        std::set<std::vector<double>> visited;

        while (!goal) {
            bool value_decreased = false;
            std::vector<double> muscle_activities = best_intermediate_result;

            std::vector<std::vector<double>> combinations = get_pattern_search_combinations(muscle_activities, step_size);

            std::cout << "step_size: " << step_size << "\n";

            for (auto muscle_activities : combinations) {

                /*std::cout << "muscles: ";
                for (auto el : muscle_activities) {
                    std::cout << el << " ";
                }
                std::cout << std::endl;*/
                
                
                if (visited.find(muscle_activities) != visited.end()) {
                    //std::cout << "--> denied" << std::endl;
                    continue;
                }
                
                

                controller.setActivation(muscle_activities);

                intermed_state = simulate_until_stop(best_state);

                current_angles = get_current_angles(intermed_state, jointangles);
                current_differences = get_current_differences(intermed_state, jointangles, current_angles);

                if (total_error_got_better(current_differences, min_differences)) {

                    std::cout << "-----------> New minimal angle: ";
                    for (int i = 0; i < jointangles.size(); ++i) {
                        std::cout << convertRadiansToDegrees(model.getJointSet().get(std::get<0>(jointangles[i])).getCoordinate().getValue(intermed_state)) << " ";
                    }
                    std::cout << std::endl;

                    best_state = intermed_state;
                    min_differences = current_differences;
                    best_angles = current_angles;
                    best_intermediate_result = muscle_activities;

                    value_decreased = true;

                    break; //if we take first improvement
                }
                visited.insert(muscle_activities);
            }

            if (!value_decreased && step_size < accuracy) {
                final_muscle_activations = best_intermediate_result;
                return std::make_tuple(final_muscle_activations, best_angles);
            }

            if (value_decreased) {
                step_size *= 2;
            }
            else {
                step_size /= 2;
            }

            if (step_size > 0.05) {
                step_size = 0.05;
            }

            if (step_size < accuracy) this->gradient_stepsize = accuracy;
        }
    }
    
    std::vector<double> get_current_differences(State& state, std::vector<std::tuple<std::string, double>>& jointangles, std::vector<double> &current_angles) {
        std::vector<double> current_differences;

        for (int i = 0; i < jointangles.size(); ++i) {
            double goal_angle = std::get<1>(jointangles[i]);
            current_differences.push_back(std::abs(goal_angle - current_angles[i]));
            //std::cout << "current difference: " << goal_angle - current_angles[i] << "\n";
        }
        return current_differences;
    }

    std::vector<double> get_current_angles(State& state, std::vector<std::tuple<std::string, double>>& jointangles) {
        std::vector<double> current_angles;

        for (int i = 0; i < jointangles.size(); ++i) {
            std::cout << "monitor angles: " << std::get<0>(jointangles[i]) << std::endl;
            auto current_angle = convertRadiansToDegrees(model.getJointSet().get(std::get<0>(jointangles[i])).getCoordinate().getValue(state));
            std::cout << "current angle: " << current_angle << std::endl;
            current_angles.push_back(current_angle);
        }
        //std::cout << "\n";
        return current_angles;
    }

    template<typename MuscleList, typename ActivationList = std::vector<double>>
    std::vector<std::tuple<std::string, double >> create_muscle_name_activation_list(MuscleList muslis, ActivationList aclis) {
        if (muslis.getSize() != aclis.size()) {
            throw "something went horribly wrong";
        }

        std::vector<std::tuple<std::string, double>> muscle_name_activity_list;

        for (int i = 0; i < muslis.getSize(); ++i) {
            muscle_name_activity_list.push_back(std::make_tuple(muslis.get(i).getName(), aclis[i]));
        }

        return muscle_name_activity_list;
    }

    /// <summary>
    /// checks, if we reached a goal state in case of stochastic search
    /// </summary>
    /// <param name="certainity_values"></param>
    /// <returns></returns>
    bool is_certain(std::deque<std::vector<double>> certainity_values) {
        bool check = true;
        double delta = this->certainity_error_delta;

        for (int i = 0; i < certainity_values[0].size(); ++i) {
            for (int j = 0; j < certainity_values.size() - 1; ++j) {
                if ((certainity_values[j][i] + delta < certainity_values[certainity_values.size() - 1][i]) || (certainity_values[j][i] - delta > certainity_values[certainity_values.size() - 1][i])) {
                    check = false;
                }
            }
        }
        return check;
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="current_differences"></param>
    /// <param name="previous_differences"></param>
    /// <returns></returns>
    bool any_angle_got_better(std::vector<double>current_differences, std::vector<double>previous_differences) {
        for (int i = 0; i < current_differences.size(); ++i) {
            if (current_differences[i] <= previous_differences[i]) {
                return true;
            }
        }

        return false;
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="current_differences">current angle differences of joints</param>
    /// <param name="previous_differences">currently best angle differences</param>
    /// <returns>returns true if no every angle difference in <c>current_differences</c> is better than in <c>previous_differences</c> </returns>
    bool every_angle_got_better(std::vector<double>current_differences, std::vector<double>previous_differences) {
        for (int i = 0; i < current_differences.size(); ++i) {
            if (current_differences[i] >= previous_differences[i]) {
                return false;
            }
        }

        return true;
    }

    bool total_error_got_better(std::vector<double>current_differences, std::vector<double>previous_differences) {
        auto current = std::accumulate(current_differences.begin(), current_differences.end(), 0.0);
        auto previous = std::accumulate(previous_differences.begin(), previous_differences.end(), 0.0);

        //std::cout << "current: " << current << " " << "previous: " << previous << std::endl;

        if (current <= previous) {
            return true;
        }
        return false;
    }

    //simulate until the muscles are not moving anymore
    State& simulate_until_stop(State& initial_state) {
        int angle_certainity_count = 5;
        std::deque<std::vector<double>> angle_certainity(angle_certainity_count - 2, std::vector<double>(model.getJointSet().getSize(), std::numeric_limits<double>::max()));
        angle_certainity.push_back(std::vector<double>(model.getJointSet().getSize(), 0.0));
        angle_certainity.push_back(std::vector<double>(model.getJointSet().getSize(), 0.0));

        State& curr_state = initial_state;
        
        int abortion_counter = this->simulation_abortion_seconds / this->integration_steptime;

        while (!angle_certainity_check(angle_certainity) && abortion_counter > 0) {
            std::vector<double> angles;
            
            Manager manager(model, curr_state);
            curr_state = manager.integrate(curr_state.getTime() + this->integration_steptime);

            for (int i = 0; i < model.getJointSet().getSize(); ++i) {
                if (model.getJointSet().get(i).numCoordinates() == 1) {
                    //std::cout << convertRadiansToDegrees(model.getJointSet().get(i).getCoordinate().getValue(curr_state)) << " ";
                    angles.push_back(convertRadiansToDegrees(model.getJointSet().get(i).getCoordinate().getValue(curr_state)));
                }
            }
            //std::cout << std::endl;
            
            angle_certainity.pop_front();
            angle_certainity.push_back(angles);

            --abortion_counter;
        }
        return curr_state;
    }

    bool angle_certainity_check(std::deque<std::vector<double>> angle_certainity) {
        bool check = true;
        double delta = this->angle_error_delta;

        for (int i = 0; i < angle_certainity[0].size(); ++i) {
            for (int j = 0; j < angle_certainity.size() - 1; ++j) {
                if ( !(angle_certainity[j][i] > angle_certainity[angle_certainity.size() - 1][i] - delta && angle_certainity[j][i] < angle_certainity[angle_certainity.size() - 1][i] + delta)) {
                    check = false;
                    break;
                }
            }
        }
        
        return check;
    }

    bool accel_vel_certainity_check(std::deque<double> accel_vel_certainity) {
        bool result = true;
        for (auto val : accel_vel_certainity) {
            if (val > 0.01) {
                result = false;
            }
        }
        return result;
    }

    bool connect_model_database(const char* host, unsigned int port, const char* user, const char* password) {
        db = new ModelDatabase(host, port, user, password);
        return db->is_connected();
    }

    void set_random_attempts(int attempts) {
        if (attempts > 0) {
            this->random_attempts = attempts;
        }
    }

    void set_random_threshold(int muscle_count) {
        if (muscle_count > 0) {
            this->random_threshold = muscle_count;
        }
    }

    double get_integration_timesteps() {
        return this->integration_steptime;
    }

    void set_integration_timesteps(double seconds) {
        if (seconds > 0) {
            this->integration_steptime = seconds;
        }
    }
    
    double get_gradient_stepsize() {
        return this->gradient_stepsize;
    }

    void set_gradient_stepsize(double stepsize) {
        if (stepsize > 0.0) {
            this->gradient_stepsize = stepsize;
        }
    }

    void set_certanity(int certainity) {
        this->error_certainity = certainity;
    }

    double get_angle_error_delta() const {
        return this->angle_error_delta;
    }

    void set_angle_error_delta(double error_delta) {
        if (error_delta > 0.0) {
            this->angle_error_delta = error_delta;
        }
    }

    int get_simulation_abortion_counter() {
        return this->simulation_abortion_seconds;
    }

    void set_abortion_counter(int abortion_counter) {
        this->simulation_abortion_seconds = abortion_counter;
    }

private:
    ///<summary>Calculates a set of permutations according to the given amount of muscles</summary>
    ///<param name="muscle_count:">amount of muscles</param>
    ///<returns>Returns a vector containing vectors with permutations of all muscle activation variations</returns>
    std::vector<std::vector<int>> calc_combinations(int muscle_count) {
        std::vector<std::vector<int>> permutations;
        if (muscle_count <= this->random_threshold) {
            permutations = cartesian_product({ std::vector<int>{-1, 1} }, muscle_count);
        }
        else {
            permutations = random_activation_vectors(muscle_count, this->random_attempts);
        }

        return permutations;
    }

    std::vector<std::vector<int>> random_activation_vectors(int size_vector, int amount) {
        std::set<std::vector<int>> added;
        std::vector<std::vector<int>> rand_vecs;

        std::random_device rnd_device;
        std::mt19937 mersenne_engine{ rnd_device() };
        std::uniform_int_distribution<int> dist{ -1, 1 };

        while(rand_vecs.size() <= amount) {
            auto gen = [&dist, &mersenne_engine]() {
                return dist(mersenne_engine);
            };

            std::vector<int>vec(size_vector);
            std::generate(vec.begin(), vec.end(), gen);

            if (added.find(vec) != added.end()) {
                continue;
            }

            rand_vecs.push_back(vec);
        }

        return rand_vecs;
    }

    template<typename T>
    std::vector<std::vector<T>> cartesian_product(std::initializer_list<std::vector<T>> ilist, size_t repeat) {
        std::vector<std::vector<T>> result{ std::vector<T>() };
        std::vector<std::vector<T>> input;

        for (int i = 0; i < repeat; ++i) {
            for (auto list : ilist) {
                input.push_back(list);
            }
        }

        for (std::vector<T> list : input) {
            std::vector<std::vector<T>> tmp;
            for (auto el : list) {
                for (auto list : result) {
                    std::vector<T> innertmp{ el };
                    innertmp.insert(innertmp.end(), list.begin(), list.end());
                    tmp.push_back(innertmp);
                }
            }
            result = tmp;
        }

        return result;
    }

    std::vector<std::vector<double>> get_pattern_search_combinations(std::vector<double> input_vector, double step_size) {
        std::vector<std::vector<double>> combinations;

        for (int i = 0; i < input_vector.size(); ++i) {
            std::vector<double> tmppos = input_vector;
            std::vector<double> tmpneg = input_vector;

            if (i == 3) {
                tmppos[i] += step_size;
                tmpneg[i] -= step_size;

                if (tmppos[i] < 0.0) {
                    tmppos[i] = 0.0;
                }
                else if (tmppos[i] > 0.2) {
                    tmppos[i] = 0.2;
                }

                if (tmpneg[i] < 0.0) {
                    tmpneg[i] = 0.0;
                }
                else if (tmpneg[i] > 0.2) {
                    tmpneg[i] = 0.2;
                }

                combinations.push_back(tmppos);
                combinations.push_back(tmpneg);

                continue;
            }

            tmppos[i] += step_size;
            if (tmppos[i] < 0.0) {
                tmppos[i] = 0.0;
            }
            else if(tmppos[i] > 1.0){
                tmppos[i] = 1.0;
            }
            
            tmpneg[i] -= step_size;
            if (tmpneg[i] < 0.0) {
                tmpneg[i] = 0.0;
            }
            else if (tmpneg[i] > 1.0) {
                tmpneg[i] = 1.0;
            }

            combinations.push_back(tmppos);
            combinations.push_back(tmpneg);
        }

        return combinations;
    }

private:
    Model& model;
    State& initial_state;
    VariableConstController& controller;
    double integration_steptime = 0.05;
    double gradient_stepsize = 0.001;
    double angle_error_delta = 0.01;
    int simulation_abortion_seconds = 100000;
    double error_certainity = 5;
    double certainity_error_delta = 0.01;
    //threshold in which the method will switch to a stochastic approach
    int random_threshold = 7;
    int random_attempts = 20;
    ModelDatabase* db = nullptr;
};
