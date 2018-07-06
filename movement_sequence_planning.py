# coding=utf-8
import networkx as nx
import collections
import matplotlib.pyplot as plt
from itertools import combinations, permutations
import copy
from skimage import img_as_bool, io, color, morphology
import matplotlib.patches as patches
import numpy as np

global VEHICLE_LENGTH
VEHICLE_LENGTH = 10


class My_graph(nx.Graph):

    # initialize edges on graph
    def add_edges(self, edges_list):
        self.edges_list = edges_list
        for edge in edges_list:
            self.add_edges_from([(edge.front_node, edge.back_node, {'edge_object': edge})])

    # initialize vehicles' paths
    def find_shortest_path(self, start_di_edge, end_di_edge):
        start_node = start_di_edge.exit_node
        end_node = end_di_edge.entrance_node
        path_nodes = nx.shortest_path(self, start_node, end_node)
        return path_nodes

    # return a list of edges' neighbors edges
    # we might get a list which includes many invalid edges, so each edge must fulfill the requirements as follow
    # 1. neighbors edge is not in input_edges_list  2. neighbors edge does not exist in neighbors_di_edges
    # then we can append it into the list neighbors_di_edges
    def find_neighbors_edges(self, edges_list):
        # print 'neighbors_edges:'
        neighbors_di_edges = []
        for edge in edges_list:
            neighbors_di_edges = self.get_neighbors_edges_by_node(edge.front_node, edges_list, neighbors_di_edges)
            neighbors_di_edges = self.get_neighbors_edges_by_node(edge.back_node, edges_list, neighbors_di_edges)
        return neighbors_di_edges

    def get_neighbors_edges_by_node(self, node, edges_list, neighbors_di_edges):
        neighbors_edges = self.edges(node)
        for neighbors_e in neighbors_edges:
            e = self[neighbors_e[0]][neighbors_e[1]]['edge_object']
            is_middle = False
            for i in edges_list:
                if i.ID == e.ID:
                    is_middle = True
            if is_middle == False and e not in neighbors_di_edges:
                de = DiEdge(e, node)
                is_exist = False
                for ne in neighbors_di_edges:
                    if de == ne:
                        is_exist = True
                if is_exist == False:
                    neighbors_di_edges.append(de)
        return neighbors_di_edges


class Edge(object):

    def __init__(self, front_node, front_node_pos, back_node, back_node_pos, width, length):
        self.front_node = front_node
        self.front_node_pos = front_node_pos
        self.back_node = back_node
        self.back_node_pos = back_node_pos
        self.width = width
        self.length = length
        self.midpoint_x = float((front_node_pos[0] + back_node_pos[0])) / 2
        self.midpoint_y = float((front_node_pos[1] + back_node_pos[1])) / 2
        self.ID = (front_node, back_node)

    # check if two edges are equal
    def __eq__(self, other_Edge):
        return self.ID == other_Edge.ID


class DiEdge(object):

    def __init__(self, edge, entrance_node):
        self.edge = edge
        self.entrance_node = entrance_node
        self.ID = self.edge.ID
        if entrance_node == self.edge.front_node:
            self.exit_node = self.edge.back_node
        else:
            self.exit_node = self.edge.front_node

    # check if two direction edges are equal
    def __eq__(self, other_deEdge):
        return self.ID == other_deEdge.ID and self.entrance_node == other_deEdge.entrance_node and self.exit_node == other_deEdge.exit_node

    # Judge whether the two edges are conflicting
    def is_conflict(self, other_di_edge):
        return self.ID == other_di_edge and self.entrance_node == other_di_edge.exit_node and self.exit_node == other_di_edge.entrance_node

    # reverse a direction edge
    def reverse(self):
        temp = self.entrance_node
        self.entrance_node = self.exit_node
        self.exit_node = temp

    def print_di_edge(self):
        return (self.entrance_node, self.exit_node)


class Vehicle(object):

    def __init__(self, name, start_di_edge, end_di_edge, path_di_edges_list):
        self.name = name
        self.start_di_edge = start_di_edge
        self.end_di_edge = end_di_edge
        self.path_di_edges_list = path_di_edges_list
        self.initialize_path()

    # initialize vehicle's path by find_shortest_path()
    def initialize_path(self):
        global G
        path_nodes = G.find_shortest_path(self.start_di_edge, self.end_di_edge)
        self.path = Path(path_nodes, self.path_di_edges_list)

    def print_vehicle_name(self):
        return self.name


class Path(object):

    # we can initialize Path by two means
    # 1. through path_nodes
    # 2. through path_di_edges_list
    def __init__(self, path_nodes=None, path_di_edges_list=[]):
        self.path_nodes = path_nodes
        self.path_di_edges_list = path_di_edges_list

    # find conflict edges by comparing the direction edges
    def find_conflict_edges(self, other_path):
        conflict_edges = []
        conflict_edges = self.check_paths_are_conflicted(other_path, conflict_edges)
        conflict_edges = self.check_vehicle_is_on_path(other_path, conflict_edges)
        return conflict_edges

    # 1. Check whether two paths are conflicted
    def check_paths_are_conflicted(self, other_path, conflict_edges):
        for de in self.path_di_edges_list:
            for other_de in other_path.path_di_edges_list:
                if de.entrance_node == other_de.exit_node and de.exit_node == other_de.entrance_node:
                    conflict_edges.append(de.edge)
        return conflict_edges

    # 2. Check whether other_vehicle is on highest_priority_vehicle's path now
    def check_vehicle_is_on_path(self, other_path, conflict_edges):
        for other_de in other_path.path_di_edges_list:
            if self.path_di_edges_list[0].edge.ID == other_de.edge.ID:
                conflict_edges.append(self.path_di_edges_list[0].edge)
        return conflict_edges

    def find_avoidance_edges(self, conflict_edges, other_path, is_exist_feasible_avoidance_di_edges):
        global G
        neighbors_di_edges = G.find_neighbors_edges(conflict_edges)
        avoidance_di_edges = self.remove_edges_if_on_other_path(neighbors_di_edges, other_path)
        avoidance_di_edges, is_exist_feasible_avoidance_di_edges = self.remove_edge_if_is_temp_edge(avoidance_di_edges,
                                                                                                    is_exist_feasible_avoidance_di_edges)
        return avoidance_di_edges, is_exist_feasible_avoidance_di_edges

    def remove_edges_if_on_other_path(self, avoidance_di_edges, other_path):
        remove_list = []
        for de in avoidance_di_edges:
            for other_de in other_path.path_di_edges_list:
                if de.edge.ID == other_de.edge.ID:
                    remove_list.append(de)
        for rm_de in remove_list:
            for de in avoidance_di_edges:
                if de.edge.ID == rm_de.edge.ID:
                    avoidance_di_edges.remove(de)
        return avoidance_di_edges

    def remove_edge_if_is_temp_edge(self, avoidance_di_edges, is_exist_feasible_avoidance_di_edges):
        for de in avoidance_di_edges:
            if de.edge == self.path_di_edges_list[0].edge:
                avoidance_di_edges.remove(de)
                is_exist_feasible_avoidance_di_edges = True
                break
        return avoidance_di_edges, is_exist_feasible_avoidance_di_edges

    def print_path(self):
        for de in self.path_di_edges_list:
            print de.ID,


class World_state_and_actions(object):

    def __init__(self, vehicles_list, parent_status=None):
        self.log = []
        self.i = 176
        self.offset = 0
        self.vehicles_list = vehicles_list
        self.parent_status = parent_status
        self.actions_list = []
        self.vehicle_state_list = []
        self.last_update_vehicle = None
        self.last_update_di_edge_list = []
        self.draw_action_index = 0
        if parent_status == None:
            self.vehicle_state_list = []
            self.edge_queue_list = []
            self.vehicle_action_list = []
            self.initialize_vehicles_states_list(self.vehicles_list)
            self.initialize_edges_queues(self.vehicles_list)
            self.initialize_actions_list(self.vehicles_list)
        else:
            self.vehicle_state_list = copy.deepcopy(parent_status.vehicle_state_list)  # copy  copy.copy deepcopy
            self.edge_queue_list = copy.deepcopy(parent_status.edge_queue_list)
            self.vehicle_action_list = []

    def initialize_vehicles_states_list(self, vehicles_list):
        for v in vehicles_list:
            vs = VehicleState(v)
            self.vehicle_state_list.append(vs)

    def initialize_edges_queues(self, vehicles_list):
        self.initialize_empty_edges_queues()
        self.add_vehicles_into_queues(vehicles_list)

    def initialize_empty_edges_queues(self):
        global G
        for e in G.edges_list:
            eq = EdgeQueue(e)
            self.edge_queue_list.append(eq)
        # print 'edge_queue_list',self.edge_queue_list[0].edge.tuple

    def initialize_actions_list(self, vehicles_list):
        for v in vehicles_list:
            act = Vehicle_action(v)
            self.vehicle_action_list.append(act)

    def add_vehicles_into_queues(self, vehicles_list):
        for v in vehicles_list:
            for eq in self.edge_queue_list:
                if v.start_di_edge.edge == eq.edge:
                    eq.append_into_queue(v.name, v.start_di_edge.exit_node)

    def update_world_states_and_actions(self, vehicle, avoidance_di_edge, path):
        for vs in self.vehicle_state_list:
            if vs.name == vehicle.name:
                self.update_actions_list(vs, avoidance_di_edge, path)
                self.update_edge_queue_list(vs.vehicle, vs.temp_di_edge, avoidance_di_edge)
                self.update_vehicle_position_list(vs.vehicle, avoidance_di_edge)
                vs.update_path_to_avoidance_edge(avoidance_di_edge)
                avoidance_di_edge.reverse()
                vs.temp_di_edge = avoidance_di_edge
                self.update_vehicle_actions_list_to_avoidance_di_edge(vehicle, avoidance_di_edge, path)

    def release_highest_priority_vehicle(self, vehicle):
        for vs in self.vehicle_state_list:
            if vs.name == vehicle.name:
                self.remove_vehicle_in_position_list(vehicle)
                self.remove_vehicle_in_queue_list(vehicle)
                self.update_two_actions_list_to_end_di_edge(vehicle, vs)

    # create Action object and append it into actions_list
    def update_actions_list(self, vehicle_state, avoidance_di_edge, path):
        self.last_update_vehicle = vehicle_state.name
        actions_list_copy = copy.deepcopy(path.path_di_edges_list)
        avoidance_di_edge_copy = copy.deepcopy(avoidance_di_edge)
        actions_list_copy.append(avoidance_di_edge_copy)
        for i in range(1, len(actions_list_copy)):
            s_act = Action(len(self.actions_list), vehicle_state, actions_list_copy[i], actions_list_copy[i - 1])
        self.actions_list.append(s_act)
        self.last_update_di_edge_list = copy.deepcopy(actions_list_copy)

    def update_vehicle_position_list(self, vehicle, destination_di_edge):
        for vs in self.vehicle_state_list:
            if vs.vehicle.name == vehicle.name:
                vs.di_edge = destination_di_edge

    def remove_vehicle_in_position_list(self, vehicle):
        for vs in self.vehicle_state_list:
            if vs.vehicle.name == vehicle.name:
                self.vehicle_state_list.remove(vs)

    def update_edge_queue_list(self, vehicle, start_di_edge, destination_di_edge):
        for qe in self.edge_queue_list:
            if qe.edge.ID == start_di_edge.edge.ID:
                qe.queue.remove(vehicle.name)
            if qe.edge.ID == destination_di_edge.edge.ID:
                qe.append_into_queue(vehicle.name, destination_di_edge.entrance_node)

    def remove_vehicle_in_queue_list(self, vehicle):
        for qe in self.edge_queue_list:
            if qe.edge.ID == vehicle.temp_di_edge.edge.ID:
                qe.queue.remove(vehicle.name)

    def update_vehicle_actions_list_to_avoidance_di_edge(self, vehicle, avoidance_di_edge, path):
        for v_act in self.vehicle_action_list:
            if v_act.vehicle.name == vehicle.name:
                if len(path.path_di_edges_list) > 1:
                    for i in range(1, len(path.path_di_edges_list)):
                        v_act.append_di_edge(path.path_di_edges_list[i])
                v_act.append_di_edge(avoidance_di_edge)

    def update_two_actions_list_to_end_di_edge(self, vehicle, vehicle_state):
        for v_act in self.vehicle_action_list:
            if v_act.vehicle.name == vehicle.name:
                if len(v_act.vehicle.path.path_di_edges_list) > 1:
                    for i in range(1, len(v_act.vehicle.path.path_di_edges_list)):
                        v_act.append_di_edge(v_act.vehicle.path.path_di_edges_list[i])
                        s_act = Action(len(self.actions_list), vehicle_state, v_act.vehicle.path.path_di_edges_list[i],
                                       v_act.vehicle.path.path_di_edges_list[i - 1])
                        self.actions_list.append(s_act)

    def check_edge_queue_is_empty(self, edge):
        for eq in self.edge_queue_list:
            if eq.edge == edge:
                if eq.queue:
                    return False
                else:
                    return True

    def check_if_vehicle_can_pop_from_edge_queue(self, vehicle_name, edge, pop_node):
        for eq in self.edge_queue_list:
            if eq.edge == edge:
                if eq.edge.front_node == pop_node:
                    if eq.queue.popleft == vehicle_name:
                        return True
                    else:
                        return True
                else:
                    if eq.queue.pop == vehicle_name:
                        return True
                    else:
                        return True

    def check_remaining_space_in_avoidance_edge(self, avoidance_edge):
        for eq in self.edge_queue_list:
            if eq.edge == avoidance_edge:
                vehicle_count = eq.count()
                global VEHICLE_LENGTH
                if avoidance_edge.length - vehicle_count * VEHICLE_LENGTH > 0:
                    return True
                else:
                    return False

    def print_world_state(self):
        print '-----------------------------'
        for v in self.vehicle_state_list:
            print 'name:', v.name
            print 'temp_di_edge:', v.temp_di_edge.print_di_edge()
            print 'destination_di_edge:', v.destination_di_edge.print_di_edge()
            print 'path_di_edges_list:'
            for de in v.path_di_edges_list:
                print de.print_di_edge(),
            print ''
            print 'path:'
            v.path.print_path()
            print '_______________'
        for qe in self.edge_queue_list:
            if qe.queue:
                print 'edge:', qe.edge.ID, 'deque:'
                for v in qe.queue:
                    print v,
                print '_______________'
        print '-----------------------------'

    def print_successive_action_list(self):
        for a in self.actions_list:
            print a.print_action()

    def optimize_successive_action_list(self):
        self.optimized_successive_action_list = []
        self.actions_list[0].start_timing = 0
        self.actions_list[0].calculate_timing_from_start_timing()
        self.optimized_successive_action_list.append(self.actions_list[0])
        for j in range(1, len(self.actions_list)):
            temp_action = self.actions_list[j]
            reverse_list = self.optimized_successive_action_list[::-1]
            find_dependence = False
            need_traverse = False
            max_dependent_timing = 0
            for previous_action in reverse_list:
                # if the two actions is from the same vehicle.
                if temp_action.vehicle_info.name == previous_action.vehicle_info.name and previous_action.end_timing > max_dependent_timing:
                    next_action = copy.deepcopy(temp_action)
                    next_action.start_timing = previous_action.end_timing
                    next_action.calculate_timing_from_start_timing()
                    max_dependent_timing = previous_action.end_timing
                    self.optimized_successive_action_list.append(next_action)
                    find_dependence = True
                # if the next vehicle must wait outside the second road until the previous vehicle leaves.
                elif temp_action.destination_di_edge.entrance_node == previous_action.temp_di_edge.exit_node and previous_action.end_timing > max_dependent_timing:
                    next_action = copy.deepcopy(temp_action)
                    next_action.middle_timing = previous_action.middle_timing
                    next_action.calculate_timing_from_middle_timing()
                    max_dependent_timing = previous_action.middle_timing
                    self.optimized_successive_action_list.append(next_action)
                    find_dependence = True
                # if the next action does not depend on the previous action
                else:
                    need_traverse = True
                if need_traverse == False:
                    break
            # if the next action does not depend any previous action
            # means that next_action.start_timing = 0
            if find_dependence == False:
                next_action = copy.deepcopy(temp_action)
                next_action.start_timing = 0
                next_action.calculate_timing_from_start_timing()
                self.optimized_successive_action_list.append(next_action)

    def print_optimized_successive_action_list(self):
        for a in self.optimized_successive_action_list:
            print a.print_action()
        self.calculate_cost()

    def calculate_cost(self):
        self.total_time = 0
        self.turn_left = 0
        self.turn_right = 0
        global turn_left_right
        for action in self.optimized_successive_action_list:
            if action.end_timing > self.total_time:
                self.total_time = action.end_timing
            if turn_left_right.get((action.temp_di_edge.edge.ID, action.destination_di_edge.edge.ID)) == 'left':
                self.turn_left = self.turn_left + 1
            if turn_left_right.get((action.temp_di_edge.edge.ID, action.destination_di_edge.edge.ID)) == 'right':
                self.turn_right = self.turn_right + 1
        print 'total_time:', self.total_time
        print 'actions_count:', len(self.optimized_successive_action_list)
        print 'turn_left:', self.turn_left
        print 'turn_right:', self.turn_right


class VehicleState(object):

    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.name = vehicle.name
        self.temp_di_edge = vehicle.start_di_edge
        self.destination_di_edge = vehicle.end_di_edge
        self.path_di_edges_list = vehicle.path_di_edges_list
        self.path = vehicle.path

    def update_path_to_avoidance_edge(self, avoidance_di_edge):
        remove_list = []
        for i in range(0, len(self.path_di_edges_list)):
            de = self.path_di_edges_list[i]
            if de.exit_node != avoidance_di_edge.entrance_node:
                remove_list.append(de)
            else:
                if avoidance_di_edge.edge == self.path_di_edges_list[i + 1].edge:
                    for rm_de in remove_list:
                        self.path_di_edges_list.remove(rm_de)
                    self.path = Path(None, self.path_di_edges_list)
                    return
                else:
                    de_reserve = copy.deepcopy(avoidance_di_edge)
                    de_reserve.reverse()
                    self.path_di_edges_list.remove(de)
                    self.path_di_edges_list.insert(0, de_reserve)
                    self.temp_di_edge = de_reserve
                    for rm_de in remove_list:
                        self.path_di_edges_list.remove(rm_de)
                    self.path = Path(None, self.path_di_edges_list)
                    return

    def print_vehicle_name(self):
        return self.name


class EdgeQueue(object):

    def __init__(self, edge):
        self.edge = edge
        self.queue = collections.deque()

    def append_into_queue(self, vehicle, node):
        if node == self.edge.front_node:
            self.queue.appendleft(vehicle)
        else:
            self.queue.append(vehicle)


class Vehicle_action(object):

    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.start_di_edge = vehicle.start_di_edge
        self.vehicle_action_list = []
        self.vehicle_action_list.append(vehicle.start_di_edge)

    def append_di_edge(self, di_edge):
        self.vehicle_action_list.append(di_edge)


class Action(object):

    def __init__(self, order_number, vehicle_info, destination_di_edge, temp_di_edge=None):
        self.order_number = order_number
        self.vehicle_info = vehicle_info
        self.destination_di_edge = destination_di_edge
        if temp_di_edge is None:
            self.temp_di_edge = vehicle_info.temp_di_edge
        else:
            self.temp_di_edge = temp_di_edge
        global velocity
        self.time_cost_first_half = float(self.temp_di_edge.edge.length / 2 / velocity)
        self.time_cost_second_half = float(self.destination_di_edge.edge.length / 2 / velocity)
        self.start_timing = None
        self.middle_timing = None
        self.end_timing = None

    def calculate_timing_from_start_timing(self):
        self.middle_timing = float(self.start_timing + self.time_cost_first_half)
        self.end_timing = float(self.middle_timing + self.time_cost_second_half)

    def calculate_timing_from_middle_timing(self):
        self.end_timing = float(self.middle_timing + self.time_cost_second_half)

    def print_action(self):
        return (self.start_timing, self.middle_timing, self.end_timing, self.vehicle_info.name,
                self.temp_di_edge.print_di_edge(), self.destination_di_edge.print_di_edge())


class Path_planning(object):

    def __init__(self, origin_vehicles_list, origin_state_and_actions):
        self.origin_vehicles_list = origin_vehicles_list
        self.origin_state_and_actions = origin_state_and_actions
        self.states_and_actions_list = []
        self.states_and_actions_list.append(origin_state_and_actions)
        self.compute_all_priorities()

    def compute_all_priorities(self):
        temp_priority_list = self.get_priority_list(self.origin_state_and_actions.vehicle_state_list)
        for priority in temp_priority_list:
            print 'one priority...'
            temp_priority = list(priority)
            temp_new_status_list = self.compute_each_priority(temp_priority, self.origin_state_and_actions)
            if temp_new_status_list is None:
                break
            while (len(temp_priority) > 0 and temp_new_status_list != []):
                for s in temp_new_status_list:
                    temp_new_status_list = self.compute_each_priority(temp_priority, s)
                    if temp_new_status_list is None:
                        break
            if temp_new_status_list is None:
                break
            print 'There is/are', len(temp_new_status_list), 'solution(s) in this priority'
            for s in temp_new_status_list:
                print s.print_successive_action_list()
                s.optimize_successive_action_list()
            print '-------------------------'
            for s in temp_new_status_list:
                print s.print_optimized_successive_action_list()

    def compute_each_priority(self, priority_list, parent_status):
        if len(priority_list) > 1:
            new_states_and_actions_list = []
            self.print_priority_list(priority_list)
            highest_priority_vehicle_state = self.get_vehicle_state(priority_list[0].name, parent_status)
            for other_vehicle in priority_list[1::]:
                is_exist_feasible_avoidance_di_edges = False
                other_vehicle_state = self.get_vehicle_state(other_vehicle.name, parent_status)
                conflict_edges = other_vehicle_state.path.find_conflict_edges(highest_priority_vehicle_state.path)
                if conflict_edges == []:
                    if new_states_and_actions_list == []:
                        new_state_and_actions = copy.deepcopy(parent_status)
                        new_states_and_actions_list.append(new_state_and_actions)
                    continue
                temp_avoidance_di_edges, is_exist_feasible_avoidance_di_edges = other_vehicle_state.path.find_avoidance_edges(
                    conflict_edges, highest_priority_vehicle_state.path, is_exist_feasible_avoidance_di_edges)
                for temp_avoidance_di_edge in temp_avoidance_di_edges:
                    path_to_avoidance_edge = self.find_paths_to_avoidance_edge(other_vehicle_state,
                                                                               temp_avoidance_di_edge,
                                                                               self.origin_state_and_actions)
                    if path_to_avoidance_edge != None:
                        is_exist_feasible_avoidance_di_edges = True
                        if new_states_and_actions_list == []:
                            new_state_and_actions = copy.deepcopy(parent_status)
                            new_state_and_actions.update_world_states_and_actions(other_vehicle_state,
                                                                                  temp_avoidance_di_edge,
                                                                                  path_to_avoidance_edge)
                            new_states_and_actions_list.append(new_state_and_actions)
                            draw = Draw(new_state_and_actions, priority_list)
                            draw.draw_states_and_actions()
                        else:
                            for s in new_states_and_actions_list:
                                s.update_world_states_and_actions(other_vehicle_state, temp_avoidance_di_edge,
                                                                  path_to_avoidance_edge)
                                draw = Draw(s, priority_list)
                                draw.draw_states_and_actions()
                    else:
                        print 'Cannot reach the avoidance_di_edge!', temp_avoidance_di_edge.print_di_edge()
                if is_exist_feasible_avoidance_di_edges == False:
                    print 'This priority is not feasible!'
                    return None
                else:
                    if new_states_and_actions_list == []:
                        new_state_and_actions = copy.deepcopy(parent_status)
                        new_states_and_actions_list.append(new_state_and_actions)
            for s in new_states_and_actions_list:
                s.release_highest_priority_vehicle(highest_priority_vehicle_state)
                draw = Draw(s, priority_list)
                draw.draw_states_and_actions()
            priority_list.remove(priority_list[0])
            return new_states_and_actions_list
        else:
            highest_priority_vehicle_state = self.get_vehicle_state(priority_list[0].name, parent_status)
            new_states_and_actions_list = []
            new_state_and_actions = copy.deepcopy(parent_status)
            new_state_and_actions.release_highest_priority_vehicle(highest_priority_vehicle_state)
            new_states_and_actions_list.append(new_state_and_actions)
            draw = Draw(new_state_and_actions, priority_list)
            draw.draw_states_and_actions()
            priority_list.remove(priority_list[0])
            return new_states_and_actions_list

    def get_priority_list(self, vehicles_list):
        priority_list = list(permutations(vehicles_list))
        return priority_list

    def find_paths_to_avoidance_edge(self, vehicle, avoidance_edge, status):
        is_blocked = False
        di_edges_to_avoidance_edges = []
        for de in vehicle.path.path_di_edges_list:
            if de == vehicle.path.path_di_edges_list[0]:
                if status.check_if_vehicle_can_pop_from_edge_queue(vehicle.name, de.edge,
                                                                   de.exit_node) == False and check_remaining_space_in_avoidance_edge == False:
                    is_blocked = True
                    break
                di_edges_to_avoidance_edges.append(de)
                if de.exit_node == avoidance_edge.entrance_node:
                    break
            else:
                if status.check_edge_queue_is_empty(de.edge) == False:
                    is_blocked = True
                    break
                di_edges_to_avoidance_edges.append(de)
                if de.exit_node == avoidance_edge.entrance_node:
                    break
        if is_blocked == False:
            p = Path(None, di_edges_to_avoidance_edges)
            return p
        else:
            return None

    def print_priority_list(self, priority_list):
        print '-----------------------------------'
        print 'priority_list:'
        for vs in priority_list:
            print vs.print_vehicle_name(),
        print '_______________'

    def get_vehicle_state(self, vehicle_name, parent_status):
        for vehicle_state in parent_status.vehicle_state_list:
            if vehicle_state.name == vehicle_name:
                return vehicle_state

    def create_states_and_actions(self, parent_status, new_states_and_actions_list):
        new_state_and_actions = copy.deepcopy(parent_status)
        new_states_and_actions_list.append(new_state_and_actions)


class Draw(object):

    def __init__(self, world_states_and_actions, priority_list):
        self.world_states_and_actions = world_states_and_actions
        self.priority_list = priority_list
        self.log = []
        self.i = 176
        self.offset = 0

    def draw_states_and_actions(self):
        (g, city) = self.minard_graph()
        plt.figure(1, figsize=(11, 5))
        plt.clf()
        colors = ['b', 'g', 'r']
        for G in g:
            # c = colors.pop(0)
            i = 0
            c = G.color[i]
            node_size = [int(G.pop[n]) for n in G]
            nx.draw_networkx_edges(G, G.pos, edge_color=c, width=4, alpha=1)
            nx.draw_networkx_nodes(G, G.pos, node_size=node_size, nodelist=G.last, node_color=c, alpha=0.5)
            nx.draw_networkx_nodes(G, G.pos, node_size=5, node_color='k')
            global edge_dict
            position = {}
            V = nx.Graph()
            for v_s in self.world_states_and_actions.vehicle_state_list:
                position[0] = ([v_s.temp_di_edge.edge.midpoint_x, v_s.temp_di_edge.edge.midpoint_y])
                V.add_node(0)
                if v_s.name == 'a':
                    vehicle_color = 'b'
                elif v_s.name == 'b':
                    vehicle_color = 'g'
                elif v_s.name == 'c':
                    vehicle_color = 'r'
                nx.draw_networkx_nodes(V, position, node_size=node_size, node_color=vehicle_color, alpha=0.5)
            nx.draw_networkx(G1, edge_dict, node_size=700)
        # for c in city:
        # x, y = city[c]
        # plt.text(x, y, c)
        title = []
        for v in self.priority_list:
            title.append(v.name)
        plt.title(str(title))
        IMG_NAME = 'map.png'
        image = img_as_bool(color.rgb2gray(io.imread(IMG_NAME)))
        plt.imshow(image, cmap='gray', interpolation='nearest')
        plt.show()

    def minard_graph(self):
        data_list = []
        '''for act in self.world_states_and_actions.vehicle_action_list:
            data = []
            for de in act.vehicle_action_list:
                data.append([de.edge.midpoint_x, de.edge.midpoint_y, 2000])
                self.offset = self.offset + 2
                self.log.append([150, self.i, act.vehicle.name + str(de.print_di_edge())])
                self.i = self.i + 0.04
            data_list.append(data)'''
        if self.world_states_and_actions.actions_list != []:
            for i in range(self.world_states_and_actions.draw_action_index,
                           len(self.world_states_and_actions.actions_list)):
                act = self.world_states_and_actions.actions_list[i]
                data = []

                # for de in act.vehicle_action_list:
                # data.append([de.edge.midpoint_x - self.offset, de.edge.midpoint_y + self.offset, 2000])
                # self.offset = self.offset + 2
                # self.log.append([150, self.i, act.vehicle.name + str(de.print_di_edge())])
                # self.i = self.i + 0.04
                de1 = act.destination_di_edge
                de2 = act.temp_di_edge
                data.append([de2.edge.midpoint_x, de2.edge.midpoint_y, 2000, act.vehicle_info.name])
                data.append([de1.edge.midpoint_x, de1.edge.midpoint_y, 2000, act.vehicle_info.name])
                self.offset = self.offset + 2
                self.log.append([de1.edge.midpoint_x, de1.edge.midpoint_y, act.vehicle_info.name])
                self.world_states_and_actions.draw_action_index = self.world_states_and_actions.draw_action_index + 1
                data_list.append(data)

        # data1 = [[1.5,1,80000],[2.5,1,80000],[3,1.5,80000]]
        c = {}
        for line in self.log:
            x, y, name = line
            c[name] = (float(x), float(y))
        g = []
        for data in data_list:
            G = nx.Graph()
            i = 0
            G.pos = {}  # location
            G.pop = {}  # size
            G.last = []
            G.color = {}
            last = None
            for line in data:
                x, y, p, v = line
                if v == 'a':
                    G.color[i] = 'b'
                elif v == 'b':
                    G.color[i] = 'g'
                elif v == 'c':
                    G.color[i] = 'r'
                G.pos[i] = (float(x), float(y))
                G.pop[i] = int(p)
                if i == len(data) - 1:
                    G.last.append(i)
                if last is None:
                    last = i
                else:
                    G.add_edge(i, last)
                    last = i
                i = i + 1
            g.append(G)
        return g, c


e1 = Edge(1, [50, 175], 2, [150, 175], 1, 50)
e2 = Edge(2, [150, 175], 3, [225, 175], 1, 50)
e3 = Edge(3, [225, 175], 4, [325, 175], 1, 50)
e4 = Edge(4, [325, 175], 5, [450, 175], 1, 50)
e5 = Edge(3, [225, 175], 6, [225, 75], 1, 10)
e6 = Edge(4, [325, 175], 7, [325, 200], 1, 50)

de1 = DiEdge(e1, 1)
de2 = DiEdge(e2, 2)
de3 = DiEdge(e3, 3)
de4 = DiEdge(e4, 4)
de5 = DiEdge(e5, 3)
de6 = DiEdge(e6, 4)

de1_reverse = DiEdge(e1, 2)
de2_reverse = DiEdge(e2, 3)
de3_reverse = DiEdge(e3, 4)
de4_reverse = DiEdge(e4, 5)
de5_reverse = DiEdge(e5, 6)
de6_reverse = DiEdge(e6, 7)
edges_list = [e1, e2, e3, e4, e5]
di_edge_list_a = [de1, de2, de5]
di_edge_list_b = [de5_reverse, de2_reverse, de1_reverse]
di_edge_list_c = [de4_reverse, de3_reverse, de5]
di_edge_list_d = [de6_reverse, de4]

global turn_left_right
turn_left_right = {(e2.ID, e5.ID): 'left', (e3.ID, e5.ID): 'right', (e5.ID, e3.ID): 'left', (e5.ID, e2.ID): 'right'}

global G
G = My_graph()
G.add_edges(edges_list)
G1 = My_graph()
G1.add_edges(edges_list)

global velocity
velocity = 10

global edge_dict
edge_dict = {}
for edge in edges_list:
    edge_dict[edge.back_node] = edge.back_node_pos
    edge_dict[edge.front_node] = edge.front_node_pos

v1 = Vehicle('a', de1, de5, di_edge_list_a)
v2 = Vehicle('b', de5_reverse, de1_reverse, di_edge_list_b)
v3 = Vehicle('c', de4_reverse, de5, di_edge_list_c)
v4 = Vehicle('d', de6_reverse, de5, di_edge_list_d)

print 'v1 path_nodes', v1.path.path_nodes
print 'v2 path_nodes', v2.path.path_nodes
print 'v3 path_nodes', v3.path.path_nodes

print '以上是输入'
print ''

s = World_state_and_actions([v1, v2, v3])
s.print_world_state()
# s.update_world_states_and_actions(vehicle, destination_di_edge, path)
print '以上是初始化Status'
print ''

p = Path_planning([v1, v2, v3], s)






