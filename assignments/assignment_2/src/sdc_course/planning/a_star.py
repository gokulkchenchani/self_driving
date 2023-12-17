from .graph import Graph
import math


def a_star_search(graph: Graph, node_to_xy, start: int, end: int):
    """
    Returns shortest path as a list of nodes ids.

    : param graph : graph definition.
    : param node_to_xy : mapping of nodes to x and y positions.
    : param start : id of the start node.
    : param end : id of the end/target node.
    """
    open_list = [start]  # List of nodes that have not been visited yet
    closed_list = []  # List of nodes that have been visited
    came_from = {start: None}  # Keep track of best predecessor of each node
    accumulated_cost = {start: 0}  # Keep track of each node and its accumulated cost
    estimated_cost = {start: 0}  # Keep track of each node and its estimated cost

    while open_list:
        current_node = open_list[0]

        for node in open_list:
            if estimated_cost[node] < estimated_cost[current_node]:
                current_node = node

        if current_node == end:
            path = []

            while current_node != start:
                path.append(current_node)
                current_node = came_from[current_node]
                
            path.append(start)
            path.reverse()
            return path
        
        open_list.remove(current_node)
        closed_list.append(current_node)

        for child_node in graph.get_children(current_node):
            if child_node in closed_list:
                continue

            cost = graph.get_cost(current_node, child_node)

            if child_node not in open_list:
                open_list.append(child_node)

            elif accumulated_cost[current_node] + cost >= accumulated_cost[child_node]:
                continue

            came_from[child_node] = current_node
            accumulated_cost[child_node] = accumulated_cost[current_node] + cost
            estimated_cost[child_node] = accumulated_cost[child_node]+ math.sqrt((node_to_xy[child_node][0] - node_to_xy[end][0]) ** 2 
                                                                                 + (node_to_xy[child_node][1] - node_to_xy[end][1]) ** 2)
        

    return []
