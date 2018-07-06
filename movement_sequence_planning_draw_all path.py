#coding=utf-8
import networkx as nx
import collections
import matplotlib.pyplot as plt
from itertools import combinations, permutations
import copy
import matplotlib.patches as patches

class My_graph(nx.Graph):

    def add(self, edges_list):
        self.edges_list = edges_list
        for edge in edges_list:
            self.add_edges_from([(edge.front_node, edge.back_node, {'object': edge})])

    def find_path(self, origin, destination):
        origin_node = origin.exit_node
        destination_node = destination.entrance_node
        path_nodes = nx.shortest_path(self, origin_node, destination_node)
        return path_nodes

    def find_neighbors_edges(self, edges_list):
        neighbors_di_edges = []
        for edge in edges_list:
            neighbors_edges = self.edges(edge.front_node)
            for neighbors_e in neighbors_edges:
                e = self[neighbors_e[0]][neighbors_e[1]]['object']
                if e not in edges_list and e not in neighbors_di_edges:
                    de = DiEdge(e, edge.front_node)
                    is_exist = False
                    for i in neighbors_di_edges:
                        if de.is_equal(i) == True:
                            is_exist = True
                    if is_exist == False:
                        neighbors_di_edges.append(de)
            neighbors_edges = self.edges(edge.back_node)
            for neighbors_e in neighbors_edges:
                e = self[neighbors_e[0]][neighbors_e[1]]['object']
                if e not in edges_list and e not in neighbors_di_edges:
                    de = DiEdge(e, edge.back_node)
                    is_exist = False
                    for i in neighbors_di_edges:
                        if de.is_equal(i) == True:
                            is_exist = True
                    if is_exist == False:
                        neighbors_di_edges.append(de)
        return neighbors_di_edges

class Edge(object):

    def __init__(self, front_node, front_node_pos, back_node, back_node_pos, weight, width):
        self.front_node  = front_node
        self.front_node_pos = front_node_pos
        self.back_node = back_node
        self.back_node_pos = back_node_pos
        self.weight = weight
        self.width = width
        self.tuple = (front_node,back_node)
        self.midpoint_x = float((front_node_pos[0] + back_node_pos[0]))/2
        self.midpoint_y = float((front_node_pos[1] + back_node_pos[1]))/2

    def is_equal(self, other_Edge):
        if self.front_node == other_Edge.front_node and self.back_node == other_Edge.back_node:
            return True
        if self.front_node == other_Edge.back_node and self.back_node == other_Edge.front_node:
            return True
        else:
            return False

    def print_edge(self):
        return (self.front_node,self.back_node)

class DiEdge(object):

    def __init__(self, edge, entrance_node):
        self.edge = edge
        self.entrance_node = entrance_node
        self.flag = False
        if entrance_node == self.edge.front_node:
            self.exit_node = self.edge.back_node
        else:
            self.exit_node = self.edge.front_node

    def is_equal(self, other_deEdge):
        if self.entrance_node == other_deEdge.entrance_node and self.exit_node == other_deEdge.exit_node:
            return True
        else:
            return False

    def reverse(self):
        temp = self.entrance_node
        self.entrance_node = self.exit_node
        self.exit_node = temp

    def print_di_edge(self):
        return (self.entrance_node,self.exit_node)

class Vehicle(object):

    def __init__(self, name, origin_di_edge, destination_di_edge, di_edge_list, graph):
        self.name = name
        self.origin_di_edge = origin_di_edge
        self.destination_di_edge = destination_di_edge
        self.di_edge_list = di_edge_list
        self.graph = graph
        self.initialize_path()

    def initialize_path(self):
        path_nodes = self.graph.find_path(self.origin_di_edge, self.destination_di_edge)
        print 'origin_di_edge',self.origin_di_edge.entrance_node
        print 'destination_di_edge',self.destination_di_edge.entrance_node
        print 'path_nodes:',path_nodes
        self.path = Path(self.graph, path_nodes, self.di_edge_list)
        #此处的路径先采用人工输入的方式。

    def print_name(self):
        return self.name



class Path(object):

    def __init__(self, graph, path_nodes = None, di_edge_list = []):
        self.path_nodes = path_nodes
        self.di_edge_list = di_edge_list
        self.graph = graph
        #if self.di_edge_list == []:
            #G_temp = nx.DiGraph()
            #nx.add_path(G_temp, path_nodes)
            #path_edge = [e for e in G_temp.edges]
            #for edge in path_edge:
                #de = DiEdge(graph[edge[0]][edge[1]]['object'], edge[1])
                #self.di_edge_list.append(de)
            #print self.di_edge_list

    def find_avoid_edges(self, highest_vehicle):
        is_collide = True
        collision_edges = self.find_collision_edges(highest_vehicle.path)
        if collision_edges == []:
            is_collide = False
        print 'collision_edges:'
        for e in collision_edges:
            print e.print_edge(),
        print '_______________'
        print 'avoid_di_edges:'
        avoid_di_edges = self.graph.find_neighbors_edges(collision_edges)
        for de in avoid_di_edges:
            for highest_de in highest_vehicle.di_edge_list:
                print highest_de.print_di_edge()
                if de.print_di_edge() == highest_de.print_di_edge():
                    avoid_di_edges.remove(de)
        print '_______________'
        return avoid_di_edges, is_collide

    def find_collision_edges(self, other_path):
        collision_edges = []
        #print 'di_edge_list:', self.di_edge_list
        for de in self.di_edge_list:
            for other_de in other_path.di_edge_list:
                if de.entrance_node == other_de.exit_node and de.exit_node == other_de.entrance_node:
                    collision_edges.append(de.edge)
        return collision_edges


    def print_path(self):
        for de in self.di_edge_list:
            print de.print_di_edge(),
        print '___________'


class StatusAndActions(object):

    def __init__(self, vehicle_list, graph, parent_status = None):
        self.cities = []
        self.offset = 0
        self.vehicle_list = vehicle_list
        self.graph = graph
        self.parent_status = parent_status
        if parent_status == None :
            self.vehicle_info_list = []
            self.edge_queue_list = []
            self.action_list = []
            self.initialize_vehicle_edge_list(self.vehicle_list)
            self.initialize_edge_queue(self.vehicle_list, self.graph)
            self.initialize_action_list(self.vehicle_list)
        else:
            self.vehicle_info_list = copy.deepcopy(parent_status.vehicle_info_list)    #copy  copy.copy deepcopy
            self.edge_queue_list = copy.deepcopy(parent_status.edge_queue_list)
            self.action_list = []

    def initialize_vehicle_edge_list(self, vehicle_list):
        for v in vehicle_list:
            ve = VehicleInfo(v)
            self.vehicle_info_list.append(ve)
        #print 'vehicle_edge_list',self.vehicle_info_list

    def initialize_edge_queue(self, vehicle_list, graph):
        self.get_empty_edge_queue(graph)
        self.add_vehicle_into_queue(vehicle_list)

    def get_empty_edge_queue(self, graph):
        for e in graph.edges_list:
            eq = EdgeQueue(e)
            self.edge_queue_list.append(eq)
        #print 'edge_queue_list',self.edge_queue_list[0].edge.tuple

    def initialize_action_list(self, vehicle_list):
        for v in vehicle_list:
            act = Action(v)
            self.action_list.append(act)

    def add_vehicle_into_queue(self, vehicle_list):
        for v in vehicle_list:
            for eq in self.edge_queue_list:
                if v.origin_di_edge.edge == eq.edge:
                    eq.append_into_queue(v.name, v.origin_di_edge.exit_node)
                #print eq.edge.tuple,eq.queue

    def update(self, vehicle, avoid_di_edge, path):
        for v in self.vehicle_info_list:
            if v.name == vehicle.name:
                self.update_edge_queue_list(v.vehicle, v.temp_di_edge, avoid_di_edge)
                self.update_vehicle_position_list(v.vehicle, avoid_di_edge)
                v.update_path(avoid_di_edge)
                avoid_di_edge.reverse()
                v.temp_di_edge = avoid_di_edge
                for act in self.action_list:
                    if act.vehicle.name == vehicle.name:
                        if len(path.di_edge_list)>1:
                            for i in range(1,len(path.di_edge_list)):
                                act.append_di_edge(path.di_edge_list[i])
                        act.append_di_edge(avoid_di_edge)
        #self.update_action_list(vehicle, origin_di_edge, destination_di_edge, path)

    def delete_vehicle(self, vehicle):
        for v in self.vehicle_info_list:
            if v.name == vehicle.name:
                self.remove_vehicle_in_position_list(vehicle)
                self.remove_vehicle_in_queue_list(vehicle)
                for act in self.action_list:
                    if act.vehicle.name == vehicle.name:
                        if len(act.vehicle.path.di_edge_list)>1:
                            for i in range(1,len(act.vehicle.path.di_edge_list)):
                                act.append_di_edge(act.vehicle.path.di_edge_list[i])

    def update_vehicle_position_list(self, vehicle, destination_di_edge):
        for ve in self.vehicle_info_list:
            if ve.vehicle.name == vehicle.name:
                ve.di_edge = destination_di_edge

    def remove_vehicle_in_position_list(self, vehicle):
        for ve in self.vehicle_info_list:
            if ve.vehicle.name == vehicle.name:
                self.vehicle_info_list.remove(ve)

    def update_edge_queue_list(self, vehicle, origin_di_edge, destination_di_edge):
        for qe in self.edge_queue_list:
            if qe.edge.print_edge() == origin_di_edge.edge.print_edge():
                qe.queue.remove(vehicle.name)
            if qe.edge.print_edge() == destination_di_edge.edge.print_edge():
                qe.append_into_queue(vehicle.name, destination_di_edge.entrance_node)

    def remove_vehicle_in_queue_list(self, vehicle):
        for qe in self.edge_queue_list:
            if qe.edge.print_edge() == vehicle.temp_di_edge.edge.print_edge():
                qe.queue.remove(vehicle.name)

    #def update_action_list(self, vehicle, origin_di_edge, destination_di_edge, path):

    def check_edge_queue_is_empty(self, edge):
        for eq in self.edge_queue_list:
            if eq.edge == edge:
                if eq.queue:
                    return False
                else:
                    return True

    def print_status(self):
        print '-----------------------------'
        print '当前状态信息'
        for v in self.vehicle_info_list:
            print 'name:',v.name
            print 'temp_di_edge:',v.temp_di_edge.print_di_edge()
            print 'destination_di_edge:', v.destination_di_edge.print_di_edge()
            print 'di_edge_list:'
            for de in v.di_edge_list:
                print de.print_di_edge(),
            print ''
            print 'path:'
            v.path.print_path()
            print '_______________'
        for qe in self.edge_queue_list:
            if qe.queue:
                print 'edge:',qe.edge.print_edge(),'deque:'
                for v in qe.queue:
                    print v,
                print '_______________'
        print '-----------------------------'
    #def update_action_list(self, vehicle, destination_edge):
	#def initialize_origin_status(status, ):

    def minard_graph(self):
        data_list = []
        print self.action_list
        for act in self.action_list:
            data = []
            for de in act.action_list:
                data.append([de.edge.midpoint_x + self.offset, de.edge.midpoint_y + self.offset, 2000])
                self.offset = self.offset + 0.01
                i = 1
                self.cities.append([1, i,str(de.print_di_edge)])
                i = i + 0.1
            data_list.append(data)
        #data1 = [[1.5,1,80000],[2.5,1,80000],[3,1.5,80000]]
        #data2 = [[4.5,1,60000],[3.5,1,60000],[2.5,1,60000],[1.5,1,60000]]
        #data3 = [[1,2,22000],[3,3,22000]]
        c = {}
        for line in self.cities:
            x, y, name = line
            c[name] = (float(x), float(y))
        g = []
        for data in data_list:
            print data
            G = nx.Graph()
            i = 0
            G.pos = {}  # location
            G.pop = {}  # size
            G.last = []
            last = None
            for line in data:
                x, y, p = line
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

    def draw(self):
        (g, city) = self.minard_graph()
        plt.figure(1, figsize=(11, 5))
        plt.clf()
        colors = ['b', 'g', 'r']
        for G in g:
            c = colors.pop(0)
            node_size = [int(G.pop[n]) for n in G]
            nx.draw_networkx_edges(G, G.pos, edge_color=c, width=4, alpha=1)
            nx.draw_networkx_nodes(G, G.pos, node_size=node_size, nodelist=G.last, node_color=c, alpha=0.5)
            nx.draw_networkx_nodes(G, G.pos, node_size=5, node_color='k')
            pos = {1: [1, 1], 2: [2, 1], 3: [3, 1], 4: [4, 1], 5: [5, 1], 6: [3, 2]}
            nx.draw_networkx(G1, pos, node_size=700)
        for c in city:
            x, y = city[c]
        plt.show()

    def get_edge_mid_point(self, di_edge):
        x = di_edge.entrance

class VehicleInfo(object):

    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.name = vehicle.name
        self.temp_di_edge = vehicle.origin_di_edge
        self.destination_di_edge = vehicle.destination_di_edge
        self.di_edge_list = vehicle.di_edge_list
        self.graph = vehicle.graph
        self.path = vehicle.path

    def update_path(self, avoid_di_edge):
        for de in self.di_edge_list:
            if de.exit_node != avoid_di_edge.entrance_node:
                de.flag = True
            else:
                de_reserve = copy.deepcopy(avoid_di_edge)
                de_reserve.reverse()
                self.di_edge_list.remove(de)
                self.di_edge_list.insert(0,de_reserve)
                self.temp_di_edge = de_reserve
                for de in self.di_edge_list:
                    if de.flag == True:
                        self.di_edge_list.remove(de)
                self.path = Path(self.graph, None, self.di_edge_list)
                return

    def print_name(self):
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
    #copy

class Action(object):

    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.origin_di_edge = vehicle.origin_di_edge
        self.action_list = []
        self.action_list.append(vehicle.origin_di_edge)

    def append_di_edge(self, di_edge):
        self.action_list.append(di_edge)

class PathToAvoidEdge(object):

    def __init__(self, vehicle, avoid_edge, status, graph):
        self.vehicle = vehicle
        self.status = status
        self.avoid_edge = avoid_edge
        self.graph = graph
        self.find_path_on_origin_path()

class Path_planning(object):

    def __init__(self, origin_vehicle_list, origin_status, graph):
        self.origin_vehicle_list = origin_vehicle_list
        self.origin_status = origin_status
        self.graph = graph
        self.status_list = []
        self.status_list.append(origin_status)
        self.compute()

    def compute(self):
        temp_priority_list = self.get_priority_list(self.origin_status.vehicle_info_list)
        for priority in temp_priority_list:
            print '======================================================================='
            print '一种优先级情况的计算'
            temp_new_status_list = []
            temp_priority = list(priority)
            temp_new_status_list = self.compute_each_priority(temp_priority, self.origin_status)
            while(len(temp_priority)>1 and temp_new_status_list != []):
                for s in temp_new_status_list:
                    temp_new_status_list = self.compute_each_priority(temp_priority, s)
            print '该优先级得到最终方案个数：',len(temp_new_status_list)

    def compute_each_priority(self, priority_list, parent_status):
        new_status_list = []
        print '-----------------------------------'
        print 'priority_list:'
        for vi in priority_list:
            print vi.print_name(),
        print '_______________'
        for vi in parent_status.vehicle_info_list:
            if vi.name == priority_list[0].name:
                highest_priority_v = vi
        print 'highest_priority_v:',highest_priority_v.name
        for other_v in priority_list[1::]:
            print '--------------------'
            print '一辆次优先级的车'
            is_block_highest = False
            for vi in parent_status.vehicle_info_list:
                if vi.name == other_v.name:
                    other_v = vi
            print 'highest_priority_v:', highest_priority_v.name
            print 'other_v:', other_v.name,'规避最高优先级的：',highest_priority_v.name
            temp_avoid_di_edges, is_collide = other_v.path.find_avoid_edges(highest_priority_v)
            print 'temp_avoid_di_edges:'
            for de in temp_avoid_di_edges:
                print de.print_di_edge(),
            print ''
            if is_collide == False:
                print '无冲突'
                print '--------------------'
                if new_status_list == []:
                    new_status = copy.deepcopy(parent_status)
                    new_status_list.append(new_status)
                continue
            for temp_avoid_di_edge in temp_avoid_di_edges:
                print 'Path To:',temp_avoid_di_edge.print_di_edge(),'is:'
                path_to_avoid_edge = self.generate_path_to_avoid_edge(other_v, temp_avoid_di_edge, self.origin_status)
                if path_to_avoid_edge != None:
                    is_block_highest = True
                    path_to_avoid_edge.print_path()
                    if new_status_list == []:
                        new_status = copy.deepcopy(parent_status)
                        new_status.update(other_v, temp_avoid_di_edge, path_to_avoid_edge)
                        new_status.print_status()
                        new_status_list.append(new_status)
                        new_status.draw()
                    else:
                        for s in new_status_list:
                            s.update(other_v, temp_avoid_di_edge, path_to_avoid_edge)
                            print '以下是更新后的状态'
                            s.print_status()
                else:
                    print '其中一个避让区无法到达',temp_avoid_di_edge.print_di_edge()
            if is_block_highest == False:
                print '该优先级方案不可行'
                return new_status_list
        print '可行的方案数量',len(new_status_list)
        print priority_list[0]
        for s in new_status_list:
            s.delete_vehicle(highest_priority_v)
            s.draw()
        priority_list.remove(priority_list[0])
        return new_status_list

    def get_priority_list(self, vehicle_list):
        priority_list = list(permutations(vehicle_list))
        print 'priority_list',priority_list
        return priority_list

    def generate_path_to_avoid_edge(self, vehicle, avoid_edge, status):
        is_blocked = False
        di_edges_to_avoid_edges = []
        i = 0
        for de in vehicle.path.di_edge_list:
            if status.check_edge_queue_is_empty(de.edge) == False and i != 0 :
                print de.exit_node
                is_blocked = True
                break
            di_edges_to_avoid_edges.append(de)
            if de.exit_node == avoid_edge.entrance_node:
                break
            i = i + 1
        if is_blocked == False:
            p = Path(self.graph, None, di_edges_to_avoid_edges)
            return p
        else:
            return None





e1 = Edge(1, [1,1], 2, [2,1], 10, 1)
e2 = Edge(2, [2,1], 3, [3,1], 10, 1)
e3 = Edge(3, [3,1], 4, [4,1], 10, 1)
e4 = Edge(4, [4,1], 5, [5,1], 10, 1)
e5 = Edge(3, [3,1], 6, [3,2], 10, 1)

de1 = DiEdge(e1, 1)
de2 = DiEdge(e2, 2)
de3 = DiEdge(e3, 3)
de4 = DiEdge(e4, 4)
de5 = DiEdge(e5, 3)

de1_reverse = DiEdge(e1, 2)
de2_reverse = DiEdge(e2, 3)
de3_reverse = DiEdge(e3, 4)
de4_reverse = DiEdge(e4, 5)
de5_reverse = DiEdge(e5, 6)
edges_list = [e1,e2,e3,e4,e5]
di_edge_list_a = [de1, de2, de5]
di_edge_list_b = [de5_reverse, de2_reverse, de1_reverse]
di_edge_list_c = [de4_reverse, de3_reverse, de5]

G = My_graph()
G.add(edges_list)
G1 = My_graph()
G1.add(edges_list)

v1 = Vehicle('a', de1, de5, di_edge_list_a, G)
v2 = Vehicle('b', de5_reverse, de1_reverse, di_edge_list_b, G)
v3 = Vehicle('c', de4_reverse, de5_reverse, di_edge_list_c, G)
print 'v1 path_nodes',v1.path.path_nodes
print 'v2 path_nodes',v2.path.path_nodes
print 'v3 path_nodes',v3.path.path_nodes

print 'avoid path:', v1.path.find_avoid_edges(v2)
print '以上是输入'
print ''

s = StatusAndActions([v1,v2,v3], G)
s.print_status()
#s.update(vehicle, destination_di_edge, path)
print '以上是初始化Status'
print ''

p = Path_planning([v1,v2,v3],s,G)




#p2 = Path([5,4,3,2,1], G)
#p2.find_collision_edges(v1.path)

#d = collections.deque()
#if d:
#    print "111"
#a = collections.deque('a')
#if a:
#    print "aaa"

#print G.edges.data(),'start'

#print e1.is_equal(e2)

