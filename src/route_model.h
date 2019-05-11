#pragma once

#include <limits>
#include <cmath>
#include <unordered_map>
#include "model.h"
#include <iostream>
using std::vector;
using std::unordered_map;

class RouteModel : public Model {

  public:
    class Node : public Model::Node {
      public:
        // Add public Node variables and methods here.
        Node * parent = nullptr;
        float h_value = std::numeric_limits<float>::max();
        float g_value = 0.0;
        bool visited = false;
        vector<Node *> neighbors;

        Node(){}
        Node(int idx, RouteModel * search_model, Model::Node node) : Model::Node(node), parent_model(search_model), index(idx) {}
        float distance(Node other) const {
          return std::sqrt(std::pow(other.x - x, 2) + std::pow(other.y - y, 2)); // 其中的 x y是继承与Node的 x y 不用声明。
        }
      	void FindNeighbors();

      private:
        // Add private Node variables and methods here.
        int index;
        RouteModel * parent_model = nullptr;

    	Node * FindNeighbor(vector<int> node_indices);
    };
    
    // Add public RouteModel variables and methods here.
    vector<Node> & SNodes(){return m_Nodes;}
    RouteModel(const std::vector<std::byte> &xml);  
    std::vector<Node> path; // This variable will eventually store the path that is found by the A* search.
  	Node &FindClosestNode(float x, float y);
    auto & GetNodeToRoadMap() {return node_to_road;}

  private:
    // Add private RouteModel variables and methods here.
    vector<Node> m_Nodes;
    unordered_map <int, vector<const Model::Road *>> node_to_road;
    void CreateNodeToRoadHashmap();

};
