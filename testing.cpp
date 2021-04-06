#include <iostream>
#include <cstring>

int main(void)
{
	int p[3][3];
	memset(p, 1, sizeof(p));
	
	for(int i=0; i<3; i++) {
		for(int j=0; j<3; j++) {
			std::cout << p[i][j] << " ";
		}
	}
}
