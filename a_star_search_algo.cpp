#include <bits/stdc++.h>
#include <stdlib.h>
#include "a_star_search_algo.h"

#define RESET   	"\033[0m"
#define BLACK   	"\033[30m"      	/* Black */
#define RED     	"\033[31m"      	/* Red */
#define GREEN   	"\033[32m"      	/* Green */
#define YELLOW  	"\033[33m"      	/* Yellow */
#define BLUE    	"\033[34m"      	/* Blue */
#define MAGENTA 	"\033[35m"      	/* Magenta */
#define CYAN    	"\033[36m"      	/* Cyan */
#define WHITE   	"\033[37m"      	/* White */
#define BOLDBLACK   	"\033[1m\033[30m"      	/* Bold Black */
#define BOLDRED     	"\033[1m\033[31m"      	/* Bold Red */
#define BOLDGREEN   	"\033[1m\033[32m"      	/* Bold Green */
#define BOLDYELLOW  	"\033[1m\033[33m"      	/* Bold Yellow */
#define BOLDBLUE    	"\033[1m\033[34m"      	/* Bold Blue */
#define BOLDMAGENTA 	"\033[1m\033[35m"      	/* Bold Magenta */
#define BOLDCYAN    	"\033[1m\033[36m"      	/* Bold Cyan */
#define BOLDWHITE   	"\033[1m\033[37m"      	/* Bold White */


#define R 30
#define C 52

int total_itr_cnt = 0;

/***********************************  UTILITIES  *************************************/
bool operator== (const node &T1, const node &T2)
{
	if(T1.coords.first == T2.coords.first && T1.coords.second == T2.coords.second )
		return true;
	else
		return false;
}


static double heuristic_cost(node c, node n)
{
	double dist = sqrt(pow((c.coords.first - n.coords.first), 2) + pow((c.coords.second - n.coords.second), 2));
	return dist;
}

bool graph::is_not_obstacle(node a)
{
	if(map[a.coords.first][a.coords.second] == 1 && a.coords.first>=0 && a.coords.second>=0 && a.coords.first<R && a.coords.second<C)
		return true;
	else
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


bool least_cost_comp(node a, node b)
{
	return (a.est_t_cost < b.est_t_cost);
}

/***********************************************************************************/
void graph::prepare_path(void)
{
	std::cout << "Source" << std::setw(7) << "(" << src.coords.first << "," << src.coords.second << ")" << std::endl;
	std::cout << "Destination" << std::setw(2) << "(" << dest.coords.first << "," << dest.coords.second << ")" << std::endl;
	
	node* temp = &nodes[CLOSED.at(CLOSED.size()-1).coords.first][CLOSED.at(CLOSED.size()-1).coords.second];
	
	while(temp->parent != NULL)
	{
		//std::cout << "(" << temp->coords.first << "," << temp->coords.second << ")" << std::endl;
		final_path.push_back(*temp);
		temp = temp->parent; 
	}
	//std::cout << "(" << temp->coords.first << "," << temp->coords.second << ")" << std::endl;
	final_path.push_back(*temp);

	std::reverse(final_path.begin(), final_path.end());
}


bool graph::initialize_search_params(void)
{
	if(is_not_obstacle(src) && is_not_obstacle(dest))
	{
		for(int i=0; i<R; i++)
		{
			for(int j=0; j<C; j++)
			{
				node a(i,j);
				nodes[i][j] = a;
			}
		}

		nodes[src.coords.first][src.coords.second].parent = NULL;
		OPEN.push_back(nodes[src.coords.first][src.coords.second]);
		
		return true;
	}
	else
		return false;
}


bool graph::start_search(void)
{
	while(OPEN.size() != 0)
	{
		node curr = OPEN.at(0);

		OPEN.erase(OPEN.begin());
		CLOSED.push_back(curr);
		
		//std::cout << curr.coords.first << "," << curr.coords.second << "," << std::endl; 

		if(curr == dest) { 
			return true;
		}

		std::vector<node> nbr_nodes = return_all_neighbours(curr);

		for(std::vector<node>::iterator itr=nbr_nodes.begin(); itr!=nbr_nodes.end(); itr++)
		{
			
			if( std::find(CLOSED.begin(), CLOSED.end(), *itr) == CLOSED.end() && is_not_obstacle(*itr) )
			{
				double tentative_past_cost = past_cost[curr.coords.first][curr.coords.second] + heuristic_cost(curr, *itr);
				if(tentative_past_cost < past_cost[itr->coords.first][itr->coords.second])
				{
					past_cost[itr->coords.first][itr->coords.second] = tentative_past_cost;
					nodes[itr->coords.first][itr->coords.second].est_t_cost = past_cost[itr->coords.first][itr->coords.second] + heuristic_cost(dest, *itr);
					nodes[itr->coords.first][itr->coords.second].p_coords.first = curr.coords.first;
					nodes[itr->coords.first][itr->coords.second].p_coords.second = curr.coords.second;

					nodes[itr->coords.first][itr->coords.second].parent = &nodes[curr.coords.first][curr.coords.second];
			
					OPEN.push_back(nodes[itr->coords.first][itr->coords.second]);
					std::sort(OPEN.begin(), OPEN.end(), least_cost_comp);
					total_itr_cnt++;
				}
			}
		}
	}


	return false;
}


void graph::print_graph_array(int arr[][C], std::vector<node> p)
{
	for(int i=0; i< R; i++) {
		for (int j=0; j<C; j++) {
			node a(i,j);
			if(std::find(p.begin(), p.end(), a) != p.end())
				std::cout << BOLDGREEN;
			else
				std::cout << RESET;
			std::cout << " " << arr[i][j];
		}
		std::cout << std::endl;
	}
}


std::vector<node> path_planning_fn(std::pair<int, int> si, std::pair<int, int> di)
{
	int map_ext[R][C] = 
{{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, }, 
 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, }, 
 {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, }, 
 {0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, }, 
 {0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, }, 
 {0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, }, 
 {0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, }, 
 {0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, }, 
 {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, }, 
 {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, }, 
 {0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, }, 
 {0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, }, 
 {0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, }, 
 {0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, }, 
 {0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, }, 
 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, }, 
 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, }, 
 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, }, 
 {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, }, 
 {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, }, 
 {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, }, 
 {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, }, 
 {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, }, 
 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, }, 
 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, }, 
 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, }, 
 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, }, 
 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, }, 
 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, }, 
 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, }};
 
 	node d(di.first, di.second);
	node s(si.first, si.second);

	graph astar_graph(map_ext, s, d); 



	if(astar_graph.initialize_search_params())
	{
		if(astar_graph.start_search()) {
			astar_graph.prepare_path();
		}
		else {
			std::cout << "No Path possible to reach to the destination!" << std::endl;
		}
	}
	else
	{
			std::cout << "Invalid source or destination!" << std::endl;
	}

	
	std::cout << "Total Iterations performed: " << total_itr_cnt << std::endl;
	astar_graph.print_graph_array(map_ext, astar_graph.final_path);
	return astar_graph.final_path;
}


int main(void)
{
	std::pair<int, int> s, e;
	std::cout << "Map size: " << R << "x" << C << std::endl;
	std::cout << "Start point X: ";
	std::cin >> s.first;
	std::cout << "Start point Y: ";
	std::cin >> s.second;
	std::cout << "End point X: ";
	std::cin >> e.first;
	std::cout << "End point Y: ";
	std::cin >> e.second;
	std::cout << std::endl << std::endl;

	std::vector<node> p = path_planning_fn(s, e);

}
