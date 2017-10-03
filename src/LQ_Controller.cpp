#include "LQ_Controller.h"

LQ_Controller::LQ_Controller()
{
	steer_angle_resulution = 0.01;

	max_steer_angle = 35;
	string input_file = "beta23kc32alpha.txt";
	char line[256];
	fstream infile;
	infile.open(input_file, fstream::in);

	if (!infile.is_open())
	{
		printf("Couldn't open file: %s \n LQ controller will not work \n", input_file.c_str());
		return;
	}

	float alpha_temp = 0;
	float beta2_temp = 0;
	float beta3_temp = 0;
	float kc_2 = 0;
	float kc_3 = 0;

	while (!infile.eof())
	{

		//Read each line in file
		infile.getline(line, 256);
		int test = sscanf_s(line, "%f %f %f %f %f", &beta2_temp, &beta3_temp, &kc_3, &kc_2, &alpha_temp);

		RegulatorData temp_data;

		temp_data.beta2 = beta2_temp;
		temp_data.beta3 = beta3_temp;
		temp_data.Kc_beta2 = kc_2;
		temp_data.Kc_beta3 = kc_3;
		temp_data.alpha = alpha_temp;

		regulator_data_struckt_list.push_back(temp_data);
	}

	infile.close();

	//Debug
	/*printf("Input read from inputfile: %s for reg : \n", input_file.c_str());
	printf("beta2:    beta3:    kc_beta2: kc_beta3: alpha: \n");
	vector<RegulatorData>::iterator iter;

	for (iter = regulator_data_struckt_list.begin(); iter != regulator_data_struckt_list.end(); iter++)
	{
	printf("%f %f %f %f %f \n", iter->beta2, iter->beta3, iter->Kc_beta2, iter->Kc_beta3, iter->alpha);
	}*/

}

LQ_Controller::LQ_Controller(double max_steer_angle_in, string input_file)
{
	steer_angle_resulution = 0.01;
	
	max_steer_angle = max_steer_angle_in;

	char line[256];
	fstream infile;
	infile.open(input_file, fstream::in);

	if (!infile.is_open())
	{
		printf("Couldn't open file: %s \n LQ controller will not work \n", input_file.c_str());
		return;
	}

	float alpha_temp = 0;
	float beta2_temp = 0;
	float beta3_temp = 0;
	float kc_2		 = 0;
	float kc_3		 = 0;

	while (!infile.eof())
	{

		//Read each line in file
		infile.getline(line, 256);
		int test = sscanf_s(line, "%f %f %f %f %f", &beta2_temp, &beta3_temp, &kc_3, &kc_2, &alpha_temp);
		
		RegulatorData temp_data;
		
		temp_data.beta2 = beta2_temp;
		temp_data.beta3 = beta3_temp;
		temp_data.Kc_beta2 = kc_2;
		temp_data.Kc_beta3 = kc_3;
		temp_data.alpha = alpha_temp;
		
		regulator_data_struckt_list.push_back(temp_data);
	}

	infile.close();

	//Debug
	/*printf("Input read from inputfile: %s for reg : \n", input_file.c_str());
	printf("beta2:    beta3:    kc_beta2: kc_beta3: alpha: \n");
	vector<RegulatorData>::iterator iter;

	for (iter = regulator_data_struckt_list.begin(); iter != regulator_data_struckt_list.end(); iter++)
	{
		printf("%f %f %f %f %f \n", iter->beta2, iter->beta3, iter->Kc_beta2, iter->Kc_beta3, iter->alpha);
	}*/

}


double LQ_Controller::Calculate_control(double desired_steer_angle, double beta2, double beta3)
{
	//printf("desired %f beta2: %f beta3: %f \n", desired_steer_angle, beta2, beta3);
	// Pick most suitable equilibrium point given a desired steer angle
	RegulatorData control_data;
	vector<RegulatorData>::iterator iter;

	for(iter = regulator_data_struckt_list.begin(); iter != regulator_data_struckt_list.end(); iter++)
	{
		if (abs(iter->alpha - desired_steer_angle) < steer_angle_resulution*0.51)
		{
			//cout << "found match! : " << iter->alpha << endl;
			control_data.alpha = iter->alpha;
			control_data.beta2 = iter->beta2;
			control_data.beta3 = iter->beta3;
			control_data.Kc_beta2 = iter->Kc_beta2;
			control_data.Kc_beta3 = iter->Kc_beta3;
		//	printf("found: %f and input: %f \n", control_data.alpha, desired_steer_angle);
			break;
		}
	}

	//u_feedback = -Lx
	double alpha_LQ = -(control_data.Kc_beta2*(beta2 - control_data.beta2) + control_data.Kc_beta3*(beta3 - control_data.beta3));

	//cout << alpha_LQ << "\n";
	//u = u_d + u_feedback
	double alpha = desired_steer_angle + alpha_LQ; 


	// Steer angle saturation: -max_steer_angle < alpha < max_steer_angle
	if( alpha > max_steer_angle )
		alpha = max_steer_angle;
	else if( alpha < -max_steer_angle )
		alpha = - max_steer_angle;

	return alpha;
}