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
    #######################################################################
    ######################### TODO: IMPLEMENT THIS ########################
    #######################################################################
    return []
