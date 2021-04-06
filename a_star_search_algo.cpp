#include <iostream>
#include <cstring>
#include <cmath>
#include <array>
#include <algorithm>
#include <vector>
#include <utility>

#include "a_star_search_algo.h"

#define R 10
#define C 10


/***********************************  UTILITIES  *************************************/
bool operator== (const node &T1, const node &T2)
{
	if(T1.coords.first == T2.coords.first && T1.coords.second == T2.coords.second )
		return true;
	else
		return false;
}


static float euclidean_cost(node c, node n)
{
	float dist = sqrt(pow((c.coords.first - n.coords.first), 2) + pow((c.coords.second - n.coords.second), 2));
	return dist;
}


bool graph::is_not_obstacle(node a)
{
	if(map[a.coords.first][a.coords.second] == 1)
		return true;
	else;
		return false;
}

static std::vector<node> return_all_neighbours(node c)
{
	std::vector<node> n;
	node n1(c.coords.first-1, c.coords.second);	//Upper
	n.push_back(n1);
	
	node n2(c.coords.first+1, c.coords.second);	//Lower
	n.push_back(n2);
	
	node n3(c.coords.first,   c.coords.second+1);	//Right
	n.push_back(n3);
	
	node n4(c.coords.first,   c.coords.second-1);	//Left
	n.push_back(n4);
	
	node n5(c.coords.first-1, c.coords.second+1);	//Upper-Right
	n.push_back(n5);
	
	node n6(c.coords.first-1, c.coords.second-1);	//Upper-Left
	n.push_back(n6);
	
	node n7(c.coords.first+1, c.coords.second+1);	//Upper-Right
	n.push_back(n7);
	
	node n8(c.coords.first+1, c.coords.second-1);	//Lower-Left
	n.push_back(n8);

	return n;
}

/***********************************************************************************/

void graph::initialize_search_params(void)
{
	OPEN.push_back(src);
	OPEN.push_back(dest);
	std::fill(past_cost.begin(), past_cost.end(), 1000);
	past_cost[0] = 0;
}


bool graph::start_search(void)
{
	while(!OPEN.empty())
	{
		float tentative_past_cost;
		node curr;

		curr = OPEN.front();
		OPEN.erase(OPEN.end());
		CLOSED.push_back(curr);

		if(curr == dest)
		{
			return true;
		}
		
		//TODO: Make 8 neighbour's list. Calculate every neighbour's est_total_cost. Add the neighbour to OPEN which has least est_total_cost.
		std::vector<node> nbr_lst = return_all_neighbours(curr);
		std::vector<float> est_total_cost;
		std::vector<node>::iterator it;
		for(std::vector<node>::const_iterator itr = nbr_lst.begin(); itr != nbr_lst.end(); itr++)
		{
			node nbr = *itr;
			it = std::find(CLOSED.begin(), CLOSED.end(), nbr); 
			if(it != CLOSED.end() && is_not_obstacle(nbr))
			{
				tentative_past_cost = past_cost[-1+((curr.coords.first)*(curr.coords.second))] + euclidean_cost(curr,nbr);
				if(tentative_past_cost < past_cost[-1+((nbr.coords.first)*(nbr.coords.second))])
				{
					past_cost[-1+((nbr.coords.first)*(nbr.coords.second))] = tentative_past_cost;
					nbr.p_coords = curr.coords;
					float etc = past_cost[-1+((nbr.coords.first)*(nbr.coords.second))] + euclidean_cost(dest, nbr);
					est_total_cost.push_back(etc);
				}
			}
		}
	}
	return false;
}



int main(int argc, char **argv)
{
	if (argc != 5) {
		std::cout << "Received: " << argc << std::endl;
	}

	int map_ext[R][C];

	node d(atoi(argv[3]), atoi(argv[4]));
	node s(atoi(argv[1]), atoi(argv[2]));

	graph astar_graph(map_ext, s, d); 
	astar_graph.initialize_search_params();
	//start_search();
}
