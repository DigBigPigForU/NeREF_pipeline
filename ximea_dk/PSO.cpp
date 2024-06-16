#include "pso.h"

void Particle_Swarm_Optimization::Init_Values(vector<vector<double>>& citys_position_0)
{	

	citys_position_ = citys_position_0;
	N = citys_position_.size();
	C = N;
	T = N;

	for (i = 0; i < N; i++)
	{
		Pbest_.push_back(100000000);
		F_.push_back(0);
		F_copy_.push_back(0);
		Gbest_individual_.push_back(0);
	}

	Gbest = 100000000;

	for (i = 0; i < N; i++)
	{
		vector<int> ini;
		for (j = 0; j < N; j++)
		{
			ini.push_back(0);
		}
		population_.push_back(ini);
		population_copy_.push_back(ini);
		Pbest_population_.push_back(ini);
	}
}

void Particle_Swarm_Optimization::PSO()
{

	Initial_Population();

	for (i = 0; i < N; i++)
	{
		F_[i] = Fitness(population_[i]);
	}

        for (it = 0; it < I_; it++)
	{

		Update_Best();

		Update_population();

		for (i = 0; i < N; i++)
		{
			F_copy_[i] = Fitness(population_copy_[i]);
		}

		for (i = 0; i < N; i++)
		{
			if (F_copy_[i] < F_[i])
			{

				for (j = 0; j < T; j++)
				{
					population_[i][j] = population_copy_[i][j];
					F_[i] = F_copy_[i];
				}
			}
			else
			{

				double r = (double)rand() / RAND_MAX;
                                if (r <= Accept_)
				{
					for (j = 0; j < T; j++)
					{
						population_[i][j] = population_copy_[i][j];
						F_[i] = F_copy_[i];
					}
				}
			}
		}


	}
}

void Particle_Swarm_Optimization::Initial_Population()
{

	vector<int> temp_city;
	for (int i = 0; i < C; i++)
	{
		temp_city.push_back(i + 1);
	}

	for (i = 0; i < N; i++)
	{
		random_shuffle(temp_city.begin(), temp_city.end());
		for (int j = 0; j < temp_city.size(); j++)
		{
			population_[i][j] = temp_city[j];
		}
	}

	population_copy_ = population_;
	cout << "init_population:" << endl;
	for (i = 0; i < N; i++)
	{
		for (j = 0; j < T; j++)
		{
			cout << population_[i][j] << " ";
		}
		cout << endl;
	}
	cout << endl;
}

double Particle_Swarm_Optimization::Fitness(vector<int>& input_solution)
{

	double cost = 0;

	for (int j = 0; j < C - 1; j++)
	{

		cost += distance(citys_position_[input_solution[j] - 1], citys_position_[input_solution[j + 1] - 1]);
	}

	cost += distance(citys_position_[input_solution[C - 1] - 1], citys_position_[input_solution[0] - 1]);

	return cost;
}

double Particle_Swarm_Optimization::distance(vector<double>& city1, vector<double>& city2)
{


	double dist = 0;
	for (int i = 0; i < city1.size(); i++)
	{
		dist += pow((city1[i] - city2[i]), 2);
	}
	dist = pow(dist, 0.5);
	return dist;
}

void Particle_Swarm_Optimization::Update_Best()
{

	for (i = 0; i < N; i++)
	{
		if (F_[i] < Pbest_[i])
		{

			Pbest_[i] = F_[i];

			for (j = 0; j < T; j++)
			{
				Pbest_population_[i][j] = population_[i][j];
			}
		}
	}

	for (i = 0; i < N; i++)
	{
		if (Pbest_[i] < Gbest)
		{

			Gbest = Pbest_[i];

			for (j = 0; j < T; j++)
			{
				Gbest_individual_[j] = Pbest_population_[i][j];
			}
		}
	}

	cout << "No." << it << "literation best solution:";
	for (j = 0; j < T; j++)
	{
		cout << Gbest_individual_[j] << "-->";
	}
	cout << Gbest_individual_[0] << endl;
	cout << "Fitness:" << Gbest << endl << endl;
}

void Particle_Swarm_Optimization::Update_population()
{


        if (c1_ != 0)
	{

		Pbest_Crossover();
	}
        if (c2_ != 0)
	{

		Gbest_Crossover();
	}

	Mutation();
}

void Particle_Swarm_Optimization::Pbest_Crossover()
{
	for (i = 0; i < N; i++)
	{

                double r1 = rand() / (double)RAND_MAX *(Upper_ - Lower_) + Lower_;

                double Pbest_CrossRate = c1_ * r1;

		double r = (double)rand() / RAND_MAX;
		if (r <= Pbest_CrossRate)
		{

			Order_Crossover(population_copy_[i], Pbest_population_[i], i);
		}
		else
		{

		}
	}
}

void Particle_Swarm_Optimization::Order_Crossover(vector<int>& father, vector<int>& mother, int k)
{
	int cut_point1, cut_point2;

	vector<int> child_indiv1 ;
	for (i = 0; i < T; i++)
	{
		child_indiv1.push_back(0);
	}

	cut_point1 = rand() % (T - 1);	//cutpoint[5]={0,1..T-3}
	cut_point2 = rand() % T;		//cutpoint[5]={0,1,2..T-2}
	while (cut_point1 >= cut_point2)
	{
		cut_point2 = rand() % T;	//cut_point2>cut_point1
	}

	for (int x = cut_point1; x <= cut_point2; x++)
	{
		child_indiv1[x] = father[x];
	}

	if (cut_point1 != 0)
	{

		int index1 = 0;
		for (int y = 0; y < T; y++)	
		{
			bool bt = true;
			for (int z = cut_point1; z <= cut_point2; z++)
			{

				if (mother[y] == child_indiv1[z])
				{
					bt = false;	

					break;
				}
			}

			if (bt == true)
			{
				child_indiv1[index1] = mother[y];
				index1 += 1;

				if (index1 == cut_point1)
				{

					if (cut_point2 != T - 1)
					{
						index1 = cut_point2 + 1;
					}

					else
					{
						break;
					}
				}
			}
		}

		for (j = 0; j < T; j++)
		{
			population_copy_[k][j] = child_indiv1[j];
		}

	}

	else
	{

		if (cut_point2 == T - 1)
		{

			for (j = 0; j < T; j++)
			{
				population_copy_[k][j] = father[j];
			}
		}

		else
		{

			int index1 = cut_point2 + 1;
			for (int y = 0; y < T; y++)
			{
				bool bt = true;
				for (int z = cut_point1; z <= cut_point2; z++)
				{

					if (mother[y] == child_indiv1[z])
					{
						bt = false;	

						break;
					}
				}

				if (bt == true)
				{
					child_indiv1[index1] = mother[y];
					index1 += 1;

					if (index1 != T)
					{
						index1 += 0;
					}

					else
					{
						break;
					}
				}
			}

			for (j = 0; j < T; j++)
			{
				population_copy_[k][j] = child_indiv1[j];
			}
		}
	}

}

void Particle_Swarm_Optimization::Gbest_Crossover()
{
	for (i = 0; i < N; i++)
	{

                double r2 = rand() / (double)RAND_MAX *(Upper_ - Lower_) + Lower_;

                double Gbest_CrossRate = c2_ * r2;

		double r = (double)rand() / RAND_MAX;
		if (r <= Gbest_CrossRate)
		{

			Order_Crossover(population_copy_[i], Gbest_individual_, i);
		}
		else
		{

		}
	}


}

void Particle_Swarm_Optimization::Mutation()
{
        double MutaRate = w_;

	int point1, point2;	
	for (i = 0; i < N; i++)
	{
		double r = { 0.0 };
		r = (double)rand() / RAND_MAX;	

		if (r <= MutaRate)
		{
			point1 = rand() % T;
			point2 = rand() % T;
			while (point1 == point2)
			{
				point2 = rand() % T;
			}
			swap(population_copy_[i][point1], population_copy_[i][point2]);
		}
	}

}

