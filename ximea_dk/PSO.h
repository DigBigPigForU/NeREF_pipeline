#pragma once
#ifndef PSO_H
#define PSO_H

#include<iostream>
#include<stdlib.h>
#include<time.h>
#include<math.h>
#include<vector>
#include<algorithm>
#include<iterator>

#define I_ 200
#define w_ 0.1
#define c1_ 2
#define c2_ 2
#define Lower_ 0.35
#define Upper_ 0.45
#define	Accept_ 0.5

using namespace std;

class Particle_Swarm_Optimization
{
private:
	int i, j;
	int it;
	int N, C, T;
	vector<vector<int>> population_;
	vector<vector<int>> population_copy_;
	vector<double> F_;
	vector<double> F_copy_;
	vector<double> Pbest_;
	vector<vector<int>> Pbest_population_;
//	vector<int> Gbest_individual_;
	double Gbest;
	vector<vector<double>> citys_position_;
public:

	void Init_Values(vector<vector<double>>&);

	void PSO();

	void Initial_Population();

	double Fitness(vector<int>& input_solution);

	double distance(vector<double>& city1, vector<double>& city2);

	void Update_Best();

	void Update_population();

	void Pbest_Crossover();

	void Order_Crossover(vector<int>& father, vector<int>& mother, int k);

	void Gbest_Crossover();

	void Mutation();

        vector<int> Gbest_individual_;
};

#endif // PCPROC_H


//int main()
//{
//	srand((unsigned)time(NULL));

//	Particle_Swarm_Optimization PSO1;
//	vector<vector<double>> citys_position_{
//	{1304,2392,5434},{2342,3639,1715},{2177,2244,7645},{3712,1399,1243},{6488,1535,8512},
//	{3826,1556,5916},{3238,1029,2684},{4196,1004,3548},{4312,7900,7162},{4306,570,8716},{4356,6866,520}
//	};
//	PSO1.Init_Values(citys_position_);
//	PSO1.PSO();

//	system("pause");
//	return 0;
//}
