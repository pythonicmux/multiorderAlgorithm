#include <multiorder_alg/MultiorderNode.hpp>

namespace Multiorder {

/*
 * The graph of engineering quad roads is as follows: (weights are distances in meters)
 * Wean Turtle             Doherty Main Entrance
 * A--89.62--B---38.74---C---53.97---D
 * |         |        /  |
 * 32.29    31.68 49.62 34.7           
 * |         |     /     |
 * E--91.91--F---39.59---G
 * Porter               Baker
 */
const std::vector<char> MultiorderNode::nodes_({'A', 'B', 'C', 'D', 'E', 'F', 'G'});

MultiorderNode::MultiorderNode(ros::NodeHandle& nodeHandle) : nh_(nodeHandle) {
    // Initialize the graph.
    for(char node:nodes_) {
        edges_[node] = std::set<char>{};
        for(char nbr:nodes_) {
            if(node != nbr) {
                weights_[std::make_pair(node, nbr)] = -1.0;
            }
        }
    }

    edges_['A'].insert('B');
    weights_[std::pair<char, char>('A', 'B')] = 89.62;
    edges_['A'].insert('E');
    weights_[std::pair<char, char>('A', 'E')] = 91.91;

    edges_['B'].insert('A');
    weights_[std::pair<char, char>('B', 'A')] = 89.62;
    edges_['B'].insert('C');
    weights_[std::pair<char, char>('B', 'C')] = 38.74;
    edges_['B'].insert('F');
    weights_[std::pair<char, char>('B', 'F')] = 31.68;

    edges_['C'].insert('B');
    weights_[std::pair<char, char>('C', 'B')] = 38.74;
    edges_['C'].insert('D');
    weights_[std::pair<char, char>('C', 'D')] = 53.97;
    edges_['C'].insert('F');
    weights_[std::pair<char, char>('C', 'F')] = 49.62;
    edges_['C'].insert('G');
    weights_[std::pair<char, char>('C', 'G')] = 34.7;

    edges_['D'].insert('C');
    weights_[std::pair<char, char>('D', 'C')] = 53.97;

    edges_['E'].insert('A');
    weights_[std::pair<char, char>('E', 'A')] = 32.29;
    edges_['E'].insert('F');
    weights_[std::pair<char, char>('E', 'F')] = 91.91;

    edges_['F'].insert('B');
    weights_[std::pair<char, char>('F', 'B')] = 31.68;
    edges_['F'].insert('C');
    weights_[std::pair<char, char>('F', 'C')] = 49.62;
    edges_['F'].insert('E');
    weights_[std::pair<char, char>('F', 'E')] = 91.91;
    edges_['F'].insert('G');
    weights_[std::pair<char, char>('F', 'G')] = 39.59;

    edges_['G'].insert('C');
    weights_[std::pair<char, char>('G', 'C')] = 34.7;
    edges_['G'].insert('C');
    weights_[std::pair<char, char>('G', 'C')] = 39.59;
}

MultiorderNode::~MultiorderNode() {}

} // namespace Multiorder
