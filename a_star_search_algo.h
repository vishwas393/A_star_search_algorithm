#include <iostream>
#include <cstring>
#include <algorithm>
#include <list>
#include <utility>

#define R 10
#define C 10

class node {
	public:
	node(void){}
	node(int a, int b) { coords.first = a; coords.second = b; }
	std::pair<int, int> coords;
	std::pair<int, int> p_coords;	//Parent's coords
	void operator= (const node& T) { 
		coords.first = T.coords.first;
		coords.second = T.coords.second;
	}

	friend bool operator== (const node& T1, const node& T2); 
};

class graph {
	public:
	graph(int m[][C], node& a, node& b) { 
		src = a; 
		dest = b;
		for (int i=0; i<R; i++) {
			for(int j=0; j<C; j++) {
				map[i][j] = m[i][j];
				}
		}
	}
	
	int map[R][C];
	node src, dest;
	std::vector<node> OPEN;
	std::vector<node> CLOSED;
	std::array<float, R*C> past_cost;

	
	void initialize_search_params(void);
	bool start_search(void);
	bool is_not_obstacle(node a);

};

