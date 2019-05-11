#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
  start_x *= 0.01f;
  start_y *= 0.01f;
  end_x *= 0.01f;
  end_y *= 0.01f;
  start_node = &m_Model.FindClosestNode(start_x, start_y);
  end_node = &m_Model.FindClosestNode(end_x, end_y);
  
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node){
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    while(current_node->parent != nullptr){
        path_found.push_back(*current_node);
        distance += current_node->distance(*(current_node->parent));  
        current_node = current_node->parent; 
    }
    path_found.push_back(*current_node);
    distance *= m_Model.MetricScale();
    return path_found;
}

float RoutePlanner::CalculateHValue(RouteModel::Node *node){
	return node->distance(*end_node);
}

void RoutePlanner::AStarSearch(){
    RouteModel::Node *current_node = nullptr;
    start_node->visited = true;
    open_list.push_back(start_node);
    while (open_list.size() > 0){
        current_node = NextNode();
        if (CalculateHValue(current_node) == 0){
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        else{
            AddNeighbors(current_node);
        }
    }
}


bool compare(const RouteModel::Node *a, const RouteModel::Node *b){
	return a->g_value + a->h_value > b->g_value + b->h_value;
}

RouteModel::Node *RoutePlanner::NextNode(){
	RouteModel::Node *the_node;
 	std::sort(open_list.begin(), open_list.end(), compare);
  	the_node = open_list.back();	// get a refrence to the frist element.

  	open_list.pop_back();
  	return the_node;
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node){
   current_node->FindNeighbors();
   for (auto neighbor : current_node->neighbors){
       neighbor->parent = current_node;
       neighbor->g_value += current_node->g_value + current_node->distance(*neighbor);
       neighbor->h_value  = CalculateHValue(neighbor);
       neighbor->visited = true;
       open_list.push_back(neighbor);
   } 
}