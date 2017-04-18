#ifndef _SETS_
#define _SETS_

#include <vector>

const int NO_VAL=-123456789;

typedef struct node_struct{
	int coords[2];
	int came_from[2];
	int t_f;
	int t_g;
}node;


class search_set{
	private:
	std::vector<node> node_list;
	void add_node(node new_node);
	void remove_node(int coords[2]);
	public:
	//This constructor initializes an empty set, always called at the beginning
	search_set();
	//This constructor initializes a set with a node in it, always called at the beginning
	search_set(node start_node);
	//~search_set();
	void push_node(node new_node);
	node pop_best();
	node pop_requested(int coords[2]);
	node read_node(int coords[2]);
	bool check_if_in_set(int coords[2]);
	bool isempty();
};
	
#endif

