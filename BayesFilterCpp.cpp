/*
Implement Bayes filter (available in RBE500 slides to iteratively calculate the belief regarding door state
Author: Shiva Surya Lolla
Date: 8th December 2022
*/

#include<iostream>
#include<vector>
using namespace std;

/*General Bayes Filter that would work for any 'action' and 'sensor_output' as input.We start with an initial belief 'old_bel'
sensor_probabs : Sensor model that contains sensor measurement probabilities 
action_probabs: Action model that contains robot action probabilities when the robot is trying to open the door 
inactionprobabs: Action model that contains robot inaction probabilities when the robot is doing nothing 
*/
vector<double> Bayes(vector<double> old_bel, double action, double sensor_output, vector<double> sensor_probabs, vector<double> action_probabs,vector<double> inactionprobabs) {
	vector<double> new_bel_bayes;
	double new_bel_open;
	double new_bel_closed;
	double new_bel_open_corr;
	double new_bel_closed_corr;
	double new_bel_open_corr_coeff;
	double new_bel_closed_corr_coeff;
	double eta;

	if (action == 0) { //If the robot does nothing 
		new_bel_open = inactionprobabs[0] * old_bel[0]+inactionprobabs[1]*old_bel[1]; //p(open/do_nothing,open)*p(open) + p(open/do_nothing,closed)*p(closed)
		new_bel_closed = inactionprobabs[2] * old_bel[0] + inactionprobabs[3] * old_bel[1]; //p(closed/do_nothing,open)*p(open) + p(closed/do_nothing,closed)*p(closed)
	}

	if (action == 1) { //If the robot tries to open the door by pushing it 
		new_bel_open = action_probabs[0] * old_bel[0] + action_probabs[2] * old_bel[1]; //p(open/push,open)*p(open) + p(open/push,closed)*p(closed)
		new_bel_closed = action_probabs[1] * old_bel[0] + action_probabs[3] * old_bel[1]; //p(closed/push,open)*p(open) + p(closed/push,closed)*p(closed)
	}

	if (sensor_output == 0) {
		new_bel_open_corr_coeff = sensor_probabs[2] * new_bel_open; //p(sense_closed/is_open)*current_bel(open)
		new_bel_closed_corr_coeff = sensor_probabs[3] * new_bel_closed; //p(sense_closed/is_closed)*current_bel(closed)
		eta = 1 / (new_bel_open_corr_coeff+new_bel_closed_corr_coeff); //calculate the normalizer 
		new_bel_open_corr = eta * new_bel_open_corr_coeff; //correct the belief about the door being open using the normalizer and earlier belief about the door being open 
		new_bel_closed_corr = eta * new_bel_closed_corr_coeff; //correct the belief about the door being closed using normalizer and earlier belief about the door being closed 
	}

	if (sensor_output == 1) {
		new_bel_open_corr_coeff = sensor_probabs[0] * new_bel_open; //p(sense_open/is_open)*current_bel(open)
		new_bel_closed_corr_coeff = sensor_probabs[1] * new_bel_closed; //p(sense_open/is_closed)*current_bel(closed)
		eta = 1 / (new_bel_open_corr_coeff + new_bel_closed_corr_coeff); //calculate the normalizer 
		new_bel_open_corr = eta * new_bel_open_corr_coeff; //correct the belief about the door being open using the normalizer and earlier belief about the door being open 
		new_bel_closed_corr = eta * new_bel_closed_corr_coeff; //correct the belief about the door being closed using normalizer and earlier belief about the door being closed 
	}
	new_bel_bayes.push_back(new_bel_open_corr); //push the latest belief about the door being open into the belief vector being returned 
	new_bel_bayes.push_back(new_bel_closed_corr); //push the latest belief about the door being closed into the belief vector being returned 
	return new_bel_bayes; //return the new belief vector 

}

int main() {
	
	//Sensor model (as per lecture slides)
	double p_sopen_isopen = 0.6;
	double p_sclosed_isopen = 0.4;
	double p_sopen_isclosed = 0.2;
	double p_sclosed_isclosed = 0.8;
	vector<double> sensor_probabs{p_sopen_isopen,p_sopen_isclosed,p_sclosed_isopen,p_sclosed_isclosed}; //sensor model indicating the probabilities of sensor detecting correctly and incorrectly

	//Actuator_model (as per lecture slides) 
	double p_o_push_o = 1;
	double p_c_push_o = 0;
	double p_o_push_c = 0.8;
	double p_c_push_c = 0.2;
	double p_o_noth_o = 1;
	double p_c_noth_o = 0;
	double p_o_noth_c = 0;
	double p_c_noth_c = 1;
	vector<double> action_probabs{p_o_push_o,p_c_push_o,p_o_push_c,p_c_push_c}; // vector containing robot action probabilities when the robot is trying to open the door 
	vector<double> inaction_probabs{p_o_noth_o,p_o_noth_c,p_c_noth_o,p_c_noth_c }; // vector containing robot inaction probabilities when the robot does nothing 

	vector<double> bel{ 0.5,0.5 }; //initial belief of open and closed 

	vector<double> action;
	action.push_back(1); //push the door 
	action.push_back(0); //do nothing 

	vector<double> sensor_output;
	sensor_output.push_back(1); //sense open door
	sensor_output.push_back(0); //sense closed door

	//Below iterations are performed as per the requirement of the assignment 
	vector<double> iteration_1{0,0}; //action: do_nothing, measurement: closed
	vector<double> iteration_2{1,0}; //Action: open, measurement: closed
	vector<double> iteration_3{0,0}; //Action: do_nothing, measurement: closed
	vector<double> iteration_4{1,1}; //Action: open, measurement: open
	vector<double> iteration_5{0,1}; //Action: do_nothing, measurement: open

	vector<vector<double>> action_sensor_pairs{iteration_1,iteration_2,iteration_3,iteration_4,iteration_5}; //Containing all the required iteration data in a vector 

	vector<double> new_bel_bayes; //latest belief that is about to be calculated using Bayes filter 

	for (int i = 0; i < action_sensor_pairs.size();i++) { //Consider data of each iteration 
		vector<double> current_pair = action_sensor_pairs[i];
		if (new_bel_bayes.empty()) {
			new_bel_bayes = Bayes(bel, current_pair[0], current_pair[1], sensor_probabs, action_probabs, inaction_probabs); //update belief after first iteration 
		}
		else { //use this loop from second iteration because we need to further change beliefs based on the latest beliefs we have 
			new_bel_bayes = Bayes(new_bel_bayes, current_pair[0], current_pair[1], sensor_probabs, action_probabs, inaction_probabs);
		}
		for (int j = 0; j < new_bel_bayes.size(); j++) { //output belief after every iteration to the terminal 
			if (j == 0) {
				cout <<"iteration "<< i << " open with belief: " << new_bel_bayes[0]<<endl;
			}
			else {
				cout << "iteration " << i << " closed with belief: " << new_bel_bayes[1]<<endl;
			}
		}
	}

	/*Test cases (as per lecture calculations)
	new_bel_bayes = Bayes(bel, 0, 1, sensor_probabs, action_probabs, inaction_probabs);
	cout << new_bel_bayes[0] << "," << new_bel_bayes[1] << endl;
	new_bel_bayes = Bayes(new_bel_bayes, 1, 1, sensor_probabs, action_probabs, inaction_probabs);
	cout <<"after 2nd action: " << new_bel_bayes[0] << "," << new_bel_bayes[1] << endl;
	*/

}