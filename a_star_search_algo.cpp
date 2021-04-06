#include <bits/stdc++.h>
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
	if(map[a.coords.first][a.coords.second] == 1 && a.coords.first>=0 && a.coords.second>=0 && a.coords.first<R && a.coords.second<C)
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

bool graph::initialize_search_params(void)
{
	if(is_not_obstacle(src) && is_not_obstacle(dest))
	{
		OPEN.push_back(src);
		PATH.push_back(src);
		//std::fill(past_cost.begin(), past_cost.end(), 1000);
		memset(past_cost, 1, sizeof(int)*R*C);
		past_cost[src.coords.first][src.coords.second] = 0;
		return true;
	}
	std::cout << "Wrong Destination or Source Provided!" << std::endl;;
	return false;
}


bool graph::start_search(void)
{
	int cnt=0;
	while(!OPEN.empty())
	{
		float tentative_past_cost;
		node curr;

		curr = OPEN.front();
		OPEN.erase(OPEN.end());
		CLOSED.push_back(curr);
		
		

		if(curr == dest)
		{
			std::cout << "You have Reached your Destination!" << std::endl;
			return true;
		}
		
		
		std::vector<node> nbr_lst = return_all_neighbours(curr);
		std::vector<float> est_total_cost;
		//std::fill(est_total_cost.begin(), est_total_cost.end(), 1000);
		std::vector<node>::iterator it;
		for(std::vector<node>::const_iterator itr = nbr_lst.begin(); itr != nbr_lst.end(); itr++)
		{
			node nbr = *itr;
			//std::cout << std::endl << nbr.coords.first << " " << nbr.coords.second;
			it = std::find(CLOSED.begin(), CLOSED.end(), nbr); 
			if(it == CLOSED.end() && is_not_obstacle(nbr))
			{
				tentative_past_cost = past_cost[curr.coords.first][curr.coords.second] + euclidean_cost(curr,nbr);
				//std::cout << " TPC: " <<tentative_past_cost << " PC: " << past_cost[nbr.coords.first][nbr.coords.second];
				if(tentative_past_cost < past_cost[nbr.coords.first][nbr.coords.second])
				{
					past_cost[nbr.coords.first][nbr.coords.second] = tentative_past_cost;
					nbr.p_coords = curr.coords;
					float etc = past_cost[nbr.coords.first][nbr.coords.second] + euclidean_cost(dest, nbr);
					//std::cout << " etc: " << etc;
					est_total_cost.push_back(etc);
				}
				else {
					est_total_cost.push_back(100);
				}
			}
			else{
				est_total_cost.push_back(100);
			}
		}
		
		
		float tmp = *std::min_element(est_total_cost.begin(), est_total_cost.end());
		int idx = std::find(est_total_cost.begin(), est_total_cost.end(), tmp) - est_total_cost.begin();
		//std::cout << nbr_lst.at(idx).coords.first << " " <<nbr_lst.at(idx).coords.second << std::endl;
		OPEN.push_back(nbr_lst.at(idx));
		PATH.push_back(nbr_lst.at(idx));
		std::cout << std::endl << nbr_lst.at(idx).coords.first << ", " << nbr_lst.at(idx).coords.second << std::endl;
			cnt++;
	}
	return false;
}



int main(int argc, char **argv)
{
	int map_ext[R][C] = {
		{1,1,1,1,1,1,1,1,1,1},
		{1,0,0,1,1,1,1,0,0,1},
		{1,0,0,1,1,1,1,0,0,1},
		{1,1,1,1,1,1,1,1,1,1},
		{1,1,1,1,1,1,1,1,1,1},
		{1,1,1,1,1,1,1,1,1,1},
		{1,1,1,1,1,1,1,1,1,1},
		{1,0,0,1,1,1,1,0,0,1},
		{1,0,0,1,1,1,1,0,0,1},
		{1,1,1,1,1,1,1,1,1,1}
	};

	node d(atoi(argv[3]),atoi(argv[4]));
	node s(atoi(argv[1]),atoi(argv[2]));

	graph astar_graph(map_ext, s, d); 
	if(astar_graph.initialize_search_params())
	{
		astar_graph.start_search();
	}
	std::cout << "Total Nodes: " << astar_graph.PATH.size() << std::endl;
}
