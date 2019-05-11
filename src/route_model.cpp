#include "route_model.h"
#include <iostream>

RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) {
    int count = 0;
    for (Model::Node node : this->Nodes()){
        m_Nodes.push_back(Node(count, this, node));
        count++;
    }
    CreateNodeToRoadHashmap();
}

/**
 * @function: create the hashmap between node_number and to road it belongs to.
 * @parameters: void
 * @retturn: void
 */
void RouteModel::CreateNodeToRoadHashmap(){
    for(auto &road : Model::Roads()){
       if (road.type != Model::Road::Type::Footway){
           for (auto node_idx : Model::Ways()[road.way].nodes){ // find the node_idx int the way which the road belongs to
               if (node_to_road.find(node_idx) == node_to_road.end()){
                   node_to_road[node_idx] = vector<const Model::Road * > ();
               }
               node_to_road[node_idx].push_back(&road);
           }
       }
    }
}

/**
 * @function: Find current node's neigbors in all derections.
 * @parameter: void
 * @ return: none
 */
RouteModel::Node * RouteModel::Node::FindNeighbor(std::vector<int> node_indices){
    Node * closed_node = nullptr; 
    Node node;
    for (int node_index : node_indices){
        node = parent_model->SNodes()[node_index];
        if (this->distance(node) != 0 && !node.visited){
            if (closed_node == nullptr || this->distance(node) < this->distance(*closed_node))
                closed_node = &(parent_model->SNodes()[node_index]);
        }
    }
    return closed_node;
}

/**
 * @function: Find the neigbors near the current node at all roads direction.
 * @parameter: void
 * @ return: none
 */
 void RouteModel::Node::FindNeighbors(){
    for (auto &road : parent_model->node_to_road[this->index]){
            RouteModel::Node *new_neighbor = this->FindNeighbor(parent_model->Ways()[road->way].nodes);
            if (new_neighbor)
                this->neighbors.push_back(new_neighbor);
    }
}

/***
 * @function: Find current node's closest node in all neighbors
 * @paramater: the cooridination between two pionts
 * @return: the Node*
 */
RouteModel::Node &RouteModel::FindClosestNode(float x, float y){
    RouteModel::Node current_node;
  	current_node.x = x;
 	current_node.y = y;
    float mini_dist = std::numeric_limits<float>::max();
    int closest_idx;
    float dist;
    for (const Model::Road &road : Roads()){
        if (road.type != Model::Road::Type::Footway){
            for (int node_idx : Ways()[road.way].nodes){
                dist = current_node.distance(SNodes()[node_idx]);
                if (dist < mini_dist){
                    mini_dist = dist;
                    closest_idx = node_idx;
                }
            }
        }
    }
    return SNodes()[closest_idx];

}