#include <iostream>
#include <cmath>
#include <vector>

using namespace std;

pair<double,double> updateCycle(pair<double,double> param_1, pair<double,double> param_2)
{
	double new_mean = 0, new_std = 0;

	new_mean = param_1.first * pow(param_2.second , 2) + param_2.first * pow(param_1.second , 2);
	new_mean = new_mean / (pow(param_1.second , 2) + pow(param_2.second , 2));

	new_std = pow(1 / ((1/pow( param_1.second , 2))  + (1/pow( param_2.second , 2)) ) , 1/2);

	pair<double,double> result(new_mean, new_std);

	return result;
}

pair<double,double> predictCycle(pair<double,double> param_1, pair<double,double> param_2)
{
	double new_mean = 0, new_std = 0;

	new_mean = param_1.first + param_2.first;
	new_std = pow( pow( param_1.second , 2)  + pow( param_2.second , 2) , 1/2);
	cout<<new_std<<"\n";

	pair<double,double> result(new_mean, new_std);
	
	return result;
}

int main()
{
	vector<double> measurements = {5,6,7,9,10};

	vector<double> motion = {1,1,2,1,1};

	double mean = 0, std = 10000;
	double std_measurement = 4, std_motion = 2;

	pair<double,double> param_1, param_2;

	param_1.first = mean;
	param_1.second = std;	

	for(int i = 0 ; i < motion.size() ; i++)
	{
		param_2.first = measurements[i];
		param_2.second = std_measurement;

		param_1 = updateCycle(param_1, param_2);
		cout<<"Result of update cycle:       Mean: "<<param_1.first<<"   Std: "<<param_1.second<<"\n";

		param_2.first = motion[i];
		param_2.second = std_motion;

		param_1 = predictCycle(param_1, param_2);
		cout<<"Result of prediction cycle:   Mean: "<<param_1.first<<"   Std: "<<param_1.second<<"\n\n";
	}
	return 0;
}
