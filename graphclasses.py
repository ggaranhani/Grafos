import copy
import sys

class Graph(object):
    def __init__(self, keys):
        self.graph = {}
        self.vertex_count = len(keys)
        self.edge_count = 0
        self.INF = 999
        for a in keys:
            self.graph[a] = {}
    
    def add_vertex(self, vert):
        if isinstance(vert, list): 
            return('Use add_vertex_from to add more than one vertex!')
        self.graph[vert] = {}
        self.vertex_count += 1
    
    def add_vertex_from(self, vertexes):
        for vertex in vertexes:
            self.add_vertex(vertex)

    def add_edge(self, a, b, w=1):
        if(a not in self.graph or b not in self.graph):
            return("Vertexes not in the graph")
        if(b not in self.graph[a] and a not in self.graph[b]):
            self.graph[a][b] = [w]
            self.graph[b][a] = [w]
            self.edge_count += 1
        elif(w not in self.graph[a][b]):
            self.graph[a][b].append(w)
            self.graph[b][a].append(w)
            self.edge_count += 1
    
    def add_edge_from(self, edges, w=None):
        if w is None:
            for edge in edges:
                if len(edge) == 3:
                    self.add_edge(edge[0], edge[1], edge[2])
                else:
                    self.add_edge(edge[0], edge[1])
        else:
            for edge in edges:
                self.add_edge(edge[0], edge[1], w)

    def remove_vertex(self, vertex):
        keyList = list(self.graph)
        if vertex in self.graph:
            keyList.remove(vertex)
            for vert in keyList:
                if(vert in self.graph[vertex]):
                    self.remove_edge(vertex, vert)
            del self.graph[vertex]

    def remove_edge(self, a, b, w=None):
        if a in self.graph and b in self.graph:
            if w is not None:
                self.graph[a][b].remove(w)
                self.graph[b][a].remove(w)
            else:
                del self.graph[a][b]
                del self.graph[b][a]
            self.edge_count -= 1

    # def remove_edge(self, a, b, **kwargs):
    #     if a in self.graph and b in self.graph:
    #         if('w' in kwargs):
    #             w = kwargs['w']
    #         if len(self.graph[a][b]) > 1:
    #             self.graph[a][b].remove(w)
    #             self.graph[b][a].remove(w)
    #         else:
    #             del self.graph[a][b]
    #             del self.graph[b][a]
    #         self.edge_count -= 1

    def print(self):
        for a in (self.graph):
            print(f"{a}: {(self.graph[a])}")

    def print_sorted(self):
        for a in sorted(self.graph):
            print(f"{a}: {{ ", end='')
            for b in sorted(self.graph[a]):
                print(f"'{b}' ", end='')
            print('}')
            
    def graph_from_graph(self):
        V = []
        E = []
        for vert in self.graph:
            V.append(vert)
            for edge in self.graph[vert]:
                if(edge not in V):
                    E.append((vert, edge))
        return V, E

    def get_dict(self):
        return self.graph
    
    def get_edge_count(self):
        V = []
        self.edge_count = 0
        for vert in self.graph:
            V.append(vert)
            for edge in self.graph[vert]:
                if(edge not in V):
                    self.edge_count += 1
        return(self.edge_count)
    
    def get_vertex_count(self):
        return(self.vertex_count)

    def get_connected_vertexes(self, vert):
        return(self.graph[vert])

    def check_connection(self, a, b):
        if(b in self.graph[a] or a in self.graph[b]):
            return True
        else:
            return False

    def get_vertex_degree_from(self, vert):
        if(len(vert) > 1):
            return([len(self.graph[v]) for v in vert])
        return(len(self.graph[vert]))

    def get_all_vertex_degree(self):                
        alldeg = []
        for vert in self.graph:
            vdeg = 0
            for neighbour in self.graph[vert]:
                vdeg += len(self.graph[vert][neighbour])
            alldeg.append(vdeg)
        return alldeg

    def get_vertex_degree(self, vert):
        return(len(self.graph[vert]))
    
    # def dfs(self, node, target, visited=None):
    #     if visited is None:
    #         visited = set()
    #     visited.add(node)
    #     if(node == target):
    #         return visited
    #     else:
    #         for n in self.graph[node]:
    #             if n not in visited and target not in visited:
    #                 self.dfs(n, target, visited)
    #     return visited

    def dfs(self, node, target, visited=None):
        if visited is None:
            visited = []
        visited.append(node)
        if(node == target):
            return visited
        else:
            for n in self.graph[node]:
                if n not in visited and target not in visited:
                    self.dfs(n, target, visited)
        return visited

    def simple_dfs(self, node, visited):
        visited[node] = True
        for n in self.graph[node]: 
            if visited[n] == False: 
                self.simple_dfs(n, visited)
    
    def instant_insanity(self, start):
        visitedV, visitedE = [], []
        e, v = self.w_cycle(start, 0, [], [], start)
        visitedV.append(v)
        visitedE.append(e)
        a,b,w = [],[],[]
        a, b, w = v[0], v[1], e[0]
        self.remove_edge(a, b, w=w)
        e, v = self.w_cycle(start, 0, [], [], start)
        visitedV.append(v)
        visitedE.append(e)
        self.add_edge(a, b, w=w)
        if not self.check_II_solution(visitedV, visitedE):
            return "No possible solution"
        else:
            return visitedV, visitedE

    def check_II_solution(self, visitedV, visitedE):
        for _ in visitedE[0]:
            if(visitedE[0] == visitedE[1]):
                return False 
            if(visitedV[0] != visitedV[1]):
                return False 
        return True

    def w_cycle(self, V, E, visitedV, visitedE, start):
        if((V in visitedV or E in visitedE) and V != start):
            return visitedV, visitedE
        visitedV.append(V)
        if(E != 0):
            visitedE.append(E)
        for vert in self.graph[V]:
            if vert not in visitedV or (vert == start and len(visitedV)==self.vertex_count):
                for edge in self.graph[V][vert]:
                    if edge not in visitedE:
                        if(vert == start and len(visitedV) == self.vertex_count):
                            visitedV.append(vert)
                            visitedE.append(edge)
                            return visitedV, visitedE
                        else:
                            self.w_cycle(vert, edge, visitedV, visitedE, start)
                            if(len(visitedV) == self.vertex_count+1):
                                return visitedE, visitedV 
                            else:
                                visitedE.remove(edge)
                                visitedV.remove(vert)
        return visitedE, visitedV
    
    def hamiltonian_circuit(self, V=None, visitedV=None, start=None):
        if(visitedV is None):
            visitedV = []
        if(start is None):
            V = list(self.graph.keys())[0]
            start = list(self.graph.keys())[0]
        if((V in visitedV) and V != start):
            return visitedV
        visitedV.append(V)
        for vert in self.graph[V]:
            if vert not in visitedV or (vert == start and len(visitedV)==self.vertex_count):
                if(vert == start and len(visitedV) == self.vertex_count):
                    visitedV.append(vert)
                    return visitedV
                else:
                    self.hamiltonian_circuit(vert, visitedV, start)
                    if(len(visitedV) == self.vertex_count+1):
                        return visitedV 
                    else:
                        visitedV.remove(vert)
        if len(visitedV) == 1:
            return("No circuit found!")
        return visitedV

    def hamiltonian_path(self, V=None, visitedV=None):
        if(visitedV is None):
            visitedV = []
        if(V is None):
            V = list(self.graph.keys())[0]
        if((V in visitedV)):
            return visitedV
        visitedV.append(V)
        for vert in self.graph[V]:
            if vert not in visitedV:
                if(len(visitedV) == self.vertex_count):
                    visitedV.append(vert)
                    return visitedV
                else:
                    self.hamiltonian_path(vert, visitedV)
                    if(len(visitedV) == self.vertex_count):
                        return visitedV 
                    else:
                        visitedV.remove(vert)
        if len(visitedV) == 1:
            return("Graph is not hamiltonian!")
        return visitedV

    def print_simple_cycle(self, start=None):
        visited = {vert: 0 for vert in self.graph.keys()}
        if start is None:
            for vert in self.graph:
                print(f"Cycle starting on vertex {vert}")
                cycle = self.simple_cycle_util(vert, visited, vert, vert)
                if len(cycle) > 1:
                    break
        else:
            cycle = self.simple_cycle_util(start, visited, start, start)
        for v in visited:
            if visited[v]:
                print(f'{v} ')

    def get_simple_cycle(self, start=None):
        visited = {vert: 0 for vert in self.graph.keys()}
        if start is None:
            for vert in self.graph:
                print(f"Cycle starting on vertex {vert}")
                cycle = self.simple_cycle_util(vert, visited, vert, vert)
                if len(cycle) > 1:
                    return cycle 
            return cycle
        return self.simple_cycle_util(start, visited, start, start)

    def print_visited(self, visited):
        for v in visited:
            if visited[v]:
                print(f'{v} ', end='')
        print()

    def simple_cycle_util(self, V, visitedV, start, parent):
        if(visitedV[V] and V != start):
            return visitedV
        visitedV[V] += 1
        for vert in self.graph[V]:
            if not visitedV[vert]:
                self.simple_cycle_util(vert, visitedV, start, V)
                if(visitedV[start] == 2):
                    return visitedV 
                else:
                    visitedV[vert] = 0
            elif(vert == start): 
                if(parent != start):
                    visitedV[vert] += 1
                    return visitedV
        return visitedV

    def fw_matrix(self):
        mat = [[self.INF]*(max(self.graph)+1) for _ in range(max(self.graph)+1)]
        for vert in self.graph:
            for neighbour in self.graph[vert]:
                mat[vert][neighbour] = self.dijkstra(vert, neighbour)[1]
        return mat

    def floyd_warshall(self):
        mat = self.fw_matrix()
        path = [[None]*(self.vertex_count+1) for _ in range(self.vertex_count+1)]
        for i in range(len(path)):
            for j in range(len(path)):
                path[i][j] = j
        for i in range(len(path)):
            path[i][i] = i
        distance = list(map(lambda i: list(map(lambda j: j, i)), mat))
        for k in range(self.vertex_count):
            for i in range(self.vertex_count):
                for j in range(self.vertex_count):
                    # distance[i][j] = min(distance[i][j], distance[i][k] + distance[k][j])
                    # path[i][j] = path[k][j]
                    if distance[i][j] > distance[i][k] + distance[k][j]:
                        distance[i][j] = distance[i][k] + distance[k][j]
                        path[i][j] = path[i][k]
        # self.print_solution(distance)
        return distance, path

    def has_cycle(self):
        cycle = self.get_simple_cycle()
        visited = 0
        for v in cycle:
            if cycle[v]:
                visited += 1
        if visited > 1:
            return True
        else:
            return False

    def has_path(self, start, target):
        if(target not in self.dfs(start, target)):
            return False
        else:
            return True

    def check_subgraph(self, edges):
        for edge in edges:
            if(self.check_connection(edge[0], edge[1]) == False):
                return False
        return True 

    def old_is_connected(self):
        connected = True
        pivot = next(iter(self.graph))
        for vert in self.graph[pivot]:
            if((self.dfs(pivot, vert)[-1] != vert)):
                connected = False
        return connected

    def is_connected(self):
        visited = {vert: False for vert in self.graph}
        for vert in self.graph: 
            if len(self.graph[vert]) > 1: 
                self.simple_dfs(vert,visited) 
                break
        if vert == self.vertex_count-1: 
            return True
        for vert in self.graph:
            if visited[vert] == False: 
                return False
        return True

    def is_connected_bfs(self):
        start_node = list(self.graph.keys())[0]
        color = {vert: 'white' for vert in self.graph.keys()}
        color[start_node] = 'gray'
        S = [start_node]
        while len(S) != 0:
            u = S.pop()
            color[u] = 'black'
            for vert in self.graph[u]:
                if color[vert] == 'white':
                    color[vert] = 'gray'
                    S.append(vert)
        return (list(color.values()).count('black') == len(self.graph.keys()))

    def is_eulerian(self):
        if not self.is_connected():
            return False 
        for deg in self.get_all_vertex_degree():
            if deg % 2 != 0:
                return False
        return True
        
    def union(self, H):
        union = copy.deepcopy(self)
        for vert in H.graph:
            if vert in union.graph:
                for edge in H.graph[vert]:
                    if edge not in union.graph[vert]:
                        union.graph[vert][edge] = []
                    for w in H.graph[vert][edge]:
                        if w not in union.graph[vert][edge]:
                            union.graph[vert][edge].append(w)
                            union.edge_count += 1
            else:
                union.graph[vert] = H.graph[vert]
                union.vertex_count += 1
        return union

    def intersection(self, H):
        intersection = copy.deepcopy(self)
        for vert in self.graph:
            if vert in H.graph:
                for edge in self.graph[vert]:
                    if edge not in H.graph[vert]:
                        del intersection.graph[vert][edge]
                        intersection.edge_count -= 1           
            else:
                del intersection.graph[vert]
                intersection.vertex_count -= 1
        return intersection

    def direct_sum(self, H):
        dsum = copy.deepcopy(self)
        for vert in H.graph:
            if vert in dsum.graph:
                for edge in H.graph[vert]:
                    if edge not in dsum.graph[vert]:
                        dsum.graph[vert][edge] = []
                    for w in H.graph[vert][edge]:
                        if w not in dsum.graph[vert][edge]:
                            dsum.graph[vert][edge].append(w)
                            dsum.edge_count += 1
                    else:
                        del dsum.graph[vert][edge]
            else:
                dsum.graph[vert] = H.graph[vert]
                dsum.vertex_count += 1
        return dsum

    def copy(self, **kwargs):
        newGraph = copy.deepcopy(self)
        if 'v_list' in kwargs:
            v_list = kwargs['v_list']
            for vert in v_list:
                if vert in newGraph.graph:
                    newGraph.remove_vertex(vert)
        if 'e_list' in kwargs:
            e_list = kwargs['e_list']
            for edge in e_list:
                if edge[0] in newGraph.graph and edge[1] in newGraph.graph:
                    newGraph.remove_edge(edge[0], edge[1])
        return newGraph

    def vertex_fusion(self, a, b):
        if a in self.graph and b in self.graph:
            for edge in self.graph[b]:
                if edge not in self.graph[a]:
                    self.graph[a][edge] = self.graph[b][edge]
                    self.graph[edge][a] = self.graph[b][a]
            self.remove_vertex(b)

    def is_tree(self):
        if self.is_connected() and not self.has_cycle():
            return True
        else:
            return False

    def dijkstra(self, initial, end):
        shortest_paths = {initial: (None, 0)}
        vert = initial
        visited = set()
        
        while vert != end:
            visited.add(vert)
            vert_weight = shortest_paths[vert][1]

            for neighbour in self.graph[vert]:
                neig_weight = min(self.graph[vert][neighbour]) + vert_weight
                if neighbour not in shortest_paths:
                    shortest_paths[neighbour] = (vert, neig_weight)
                else:
                    if shortest_paths[neighbour][1] > neig_weight:
                        shortest_paths[neighbour] = (vert, neig_weight)
            
            not_visited = {node: shortest_paths[node] for node in shortest_paths if node not in visited}
            if not not_visited:
                return(f"Path between {initial} and {end} does not exist")
            vert = min(not_visited, key=lambda k: not_visited[k][1])
        
        path = []
        weight = shortest_paths[vert][1]
        while vert is not None:
            path.append(vert)
            vert = shortest_paths[vert][0]

        path = path[::-1]
        return path, weight

    def shortest_cycle(self, initial):
        distances = {}
        paths = {}
        for neighbour in list(self.graph[initial]):
            weight = self.graph[initial][neighbour]
            self.remove_edge(initial, neighbour)
            try:
                paths[neighbour], distances[neighbour] = self.dijkstra(neighbour, initial)
                distances[neighbour] += min(weight)
            except:
                pass
            for w in weight:
                self.add_edge(initial, neighbour, w)        
        try:
            vert = min(distances, key=lambda k: distances[k])
        except:
            return(f"There is no possible cycle for vertex {initial}")
        return paths[vert], distances[vert] 

    def eccentricity(self):
        ecc = 0
        for vert in self.graph:
            for vert2 in self.graph:
                weight = self.dijkstra(vert, vert2)[1]
                if weight > ecc:
                    ecc = weight
        return ecc
    
    def center(self):
        if self.is_tree():
            ecc = sys.maxsize
            center = []
            for vert in self.graph:
                for vert2 in self.graph:
                    weight = self.dijkstra(vert, vert2)[1]
                    if weight < ecc and (vert != vert2):
                        ecc = weight
                        center = []
                        center.append(vert)
                    elif weight == ecc and vert not in center:
                        center.append(vert)
            return center
        else:
            return "Graph is not a tree"

    # for non numerical weights
    def even_graph(self):
        for vert in list(self.graph):
            if self.get_vertex_degree(vert) % 2 != 0:
                print(vert)
                nw = {}
                for neighbour in self.graph[vert]:
                    nw[neighbour] = (min(self.graph[vert][neighbour]))
                minimal = min(nw, key = nw.get)
                self.add_edge(vert, minimal, (nw[minimal])+0.00000001)


if __name__ == '__main__':
    # G = Graph({'B', 'Y', 'R', 'G'})
    # cubo1 = [('B', 'R'), ('Y', 'G'), ('R', 'G')]
    # cubo2 = [('R', 'Y'), ('G', 'Y'), ('B', 'G')]
    # cubo3 = [('Y', 'G'), ('R', 'B'), ('G', 'R')]
    # cubo4 = [('G', 'B'), ('R', 'R'), ('Y', 'B')]
    # G.add_edge_from(cubo1, w=1)
    # G.add_edge_from(cubo2, w=2)
    # G.add_edge_from(cubo3, w=3)
    # G.add_edge_from(cubo4, w=4)
    # G.print()
    # G.instant_insanity('R')

    # G = Graph({'R', 'G', 'Y', 'B'})
    # cubo1 = [('R', 'G'), ('R', 'R'), ('B', 'Y')]
    # cubo2 = [('G', 'G'), ('B', 'Y'), ('B', 'R')]
    # cubo3 = [('R', 'Y'), ('G', 'Y'), ('R', 'B')]
    # cubo4 = [('B', 'Y'), ('G', 'Y'), ('G', 'R')]
    # G.add_edge_from(cubo1, w=1)
    # G.add_edge_from(cubo2, w=2)
    # G.add_edge_from(cubo3, w=3)
    # G.add_edge_from(cubo4, w=4)
    # G.print()
    # G.instant_insanity('B')
    # G.hamiltonian_path()
    # G.w_cycle('Y', 0, [], [], 'Y')
    # G.w_cycle('G', 0, [], [], 'G')
    # G.w_cycle('B', 0, [], [], 'B')
    # G.w_cycle('R', 0, [], [], 'R')
    # G.remove_edge('R','G',w=1)
    # G.w_cycle('R', 0, [], [], 'R')
    # G.remove_edge('R','G',w=4)
    # G.w_cycle('R', 0, [], [], 'R')
    # G.add_edge('R','G',w=1)
    # G.w_cycle('R', 0, [], [], 'R')
    # G.add_edge('R','G',w=4)
    # G.w_cycle('R', 0, [], [], 'R')



    G = Graph({1,2,3,4})
    G.add_edge_from([(1,2), (2,3), (3, 4)])
    G.add_edge_from([(1,2,1), (1,4,2), (2,3,2), (2, 4,2), (3, 4,3)])
    G.add_vertex(5)
    G.add_vertex(6)
    G.add_edge(6,5,4)
    G.add_edge(3,5,3)

    # G.print_sorted()
    # H = Graph({1,4,5,6,7})
    # H.add_edge_from([(1,7), (7,5), (4,6), (2,7), (5,1), (6,5), (1,4)])
    # H.print_sorted()

    # union = G.union(H)
    # union.print()

    # intersection = G.intersection(H)
    # intersection.print_sorted()

    # dsum = G.direct_sum(H)
    # dsum.print_sorted()

    # J = G.copy(v_list=[1], e_list=[(2,3)])
    # J.print_sorted()

    # G.vertex_fusion(1,2)



    ############################# TESTES TEMPO
    # import time
    # import timeit

    # code_to_test  = """
    # G.is_connected_bfs()
    # """
    # code_to_test2  = """
    # G.old_is_connected()
    # """
    # code_to_test3  = """
    # G.is_connected()
    # """

    # elapsed_time = timeit.timeit(code_to_test, globals=globals())/100
    # print(elapsed_time)
    # elapsed_time = timeit.timeit(code_to_test2, globals=globals())/100
    # print(elapsed_time)
    # elapsed_time = timeit.timeit(code_to_test3, globals=globals())/100
    # print(elapsed_time)

    # start = time.clock()
    # print(is_connected(G.graph))
    # print(time.clock()-start)


    # start = time.clock()
    # print(G.is_connected())
    # print(time.clock()-start)