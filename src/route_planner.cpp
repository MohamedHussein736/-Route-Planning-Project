#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    this->start_node = &m_Model.FindClosestNode(start_x, start_y);
    this->end_node = &m_Model.FindClosestNode(end_x, end_y);
}

// TODO 3: Implement the CalculateHValue method.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
	return node->distance(*(this->end_node));
}

// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
  
    // For each neighbor
    for (RouteModel::Node* neighbor : current_node->neighbors) {
        if (!neighbor->visited) {
            neighbor->parent = current_node;        
            // Calculate g Value   
            neighbor->g_value = current_node->g_value + current_node->distance(*neighbor); 
            // Calculate H Value 
            neighbor->h_value = CalculateHValue(neighbor);      
            // Add neighbor 
            this->open_list.push_back(neighbor);
            // Set the visited neighbor to true
            neighbor->visited = true; 
        }
    }
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.

// Note: here we don NOT need to explicitly specifiy "static" here
bool compareFVal(const RouteModel::Node* node_a, const RouteModel::Node* node_b) {
    // F = g + h
    return (node_a->g_value + node_a->h_value) > (node_b->g_value + node_b->h_value);
}

RouteModel::Node *RoutePlanner::NextNode() {
    // sort the open_list according to the f-value 
    std::sort(this->open_list.begin(), this->open_list.end(), compareFVal);
    // copy the pointer to the node with lowest f-value
    RouteModel::Node* lowest_f_val_node = this->open_list.back();
    this->open_list.pop_back();

    return lowest_f_val_node;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.


//wrong

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    // For each node in the path

    while (current_node->parent != nullptr) {
        
        // Add the distance
        distance += current_node->distance(*current_node->parent);
        // Store node in the path
        path_found.push_back(*current_node);

        // Move to the parent node
        current_node = current_node->parent;
    }

    // Push back initial node
    path_found.push_back(*current_node);

    // Sort path (end2start) -> (start2end)
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.
    this->start_node->visited = true;
    this->open_list.push_back(this->start_node); 

    // While the node list not empty
    while (!open_list.empty()) {

    // catch the next node
    current_node = this->NextNode();     
      if (current_node == this->end_node) {
          this->m_Model.path = this->ConstructFinalPath(current_node);
          return;
      }

      else{
          this->AddNeighbors(current_node);
      }
    }
}