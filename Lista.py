from collections import deque
import heapq
class List:

    def __init__(self, n):
        self.graph = []
        for i in range(n):
            self.graph.append([])
        self.graph = {i: self.graph[i] for i in range(len(self.graph))}

    def add_edge(self, v1, v2, weight=None, directed=False):
        self.graph[v1].append((v2, weight))
        
        if not directed:
            self.graph[v2].append((v1, weight))
    
    def remove_edge(self, v1, v2, directed=False):
        for edge in self.graph[v1]:
            if edge[0] == v2:
                self.graph[v1].remove(edge)
        
        if not directed:
            for edge in self.graph[v2]:
                if edge[0] == v1:
                    self.graph[v2].remove(edge)
    
    def get_vertice_degree(self, v, directed=False):
        if not directed:
            return len(self.graph[v])
        else:
            entry_degree = 0
            output_degree = len(self.graph[v])
            
            for i in self.graph:
                for edge in self.graph[i]:
                    if edge[0] == v:
                        entry_degree += 1

        return {"Entry": entry_degree, "Output": output_degree}

    def get_graph_degree(self, directed=False):
        if not directed:
            degree = 0
            for vertex in self.graph:
                degree += len(self.graph[vertex])
            return degree
        else:
            entry_degree = 0
            output_degree = 0

            for vertex in self.graph:
                output_degree += len(self.graph[vertex])
                
                for edge in self.graph[vertex]:
                    if vertex in edge:
                        entry_degree += 1
                
            return {"Entrie": entry_degree, "Output": output_degree}
        
    def get_vertice_neighborhood(self, v, directed=False):
        neighbors = []

        for vertex, edges in self.graph.items():
            if directed:
                if any(edge[0] == v for edge in edges):
                    neighbors.append(vertex)
            else:
                if any(edge[0] == v for edge in edges) or any(v in edge for edge in self.graph[vertex]):
                    neighbors.append(vertex)

        return neighbors

    def is_connected(self):
        if self.graph:  # Verifica se o grafo não está vazio
            start_vertex = next(iter(self.graph))  # Escolhe o primeiro vértice do grafo como ponto de partida
            visited = self.dfs_connected(start_vertex)
            return (len(visited) == len(self.graph))
        else:
            return False
        
    def dfs_connected(self, start):
        visited = []

        def dfs_helper(node):
            visited.append(node)
            for neighbor, _ in self.graph.get(node, []):
                if neighbor not in visited:
                    dfs_helper(neighbor)                
        dfs_helper(start)
        return visited

    def is_regular(self, directed=False):
        if not directed:
            degrees = [self.get_vertice_degree(v) for v in self.graph]
            return all(deg == degrees[0] for deg in degrees)
        else:
            entry_degrees = [len([v for v, _ in edges]) for edges in self.graph.values()]
            out_degrees = [self.get_vertice_degree(v) for v in self.graph]

            return all(entry == entry_degrees[0] and out == out_degrees[0] for entry, out in zip(entry_degrees, out_degrees))
    
    def is_complete(self, directed = False):
        if not directed:
            degree = len(self.graph) - 1
            for i in self.graph:
                if degree != self.get_vertice_degree(i):
                    return False
        else:
            degree = len(self.graph) - 1
            for i in self.graph:
                if degree != self.get_vertice_degree(i):
                    return False
                if i in self.graph:
                    counter += counter
                if counter != degree:
                    return False
        return True
    
    def dfs(self, start):
        visited = []

        def dfs_helper(node):
            visited.append(node)
            print(node, end="->")
            for neighbor, _ in self.graph.get(node, []):
                if neighbor not in visited:
                    dfs_helper(neighbor)

        dfs_helper(start)  # Inicie a busca DFS com o valor de start do parâmetro.
        print("")

        while len(visited) < len(self.graph):
            # Encontre um vértice não visitado como um novo ponto de partida.
            start_vertex = None
            for vertex in self.graph:
                if vertex not in visited:
                    start_vertex = vertex
                    break

            if start_vertex is not None:
                dfs_helper(start_vertex)
                print("")

        return list(visited)
    
    def bfs(self, vertice):
        visited = set()
        all_visited = set()
        queue = deque([vertice])

        while queue:
            vertex = queue.popleft()
            if vertex not in visited:
                print(vertex, end="->")
                visited.add(vertex)
                all_visited.add(vertex)
                queue.extend(neighbor for neighbor, _ in self.graph.get(vertex, []) if neighbor not in all_visited)

        # Verifica se todos os vértices foram visitados; se não, encontre um novo ponto de partida.
        while all_visited != set(self.graph.keys()):
            print("")
            unvisited_vertices = set(self.graph.keys()) - all_visited
            start_vertex = unvisited_vertices.pop()
            queue = deque([start_vertex])
            visited = set()
            while queue:
                vertex = queue.popleft()
                if vertex not in visited:
                    print(vertex, end="->")
                    visited.add(vertex)
                    all_visited.add(vertex)
                    queue.extend(neighbor for neighbor, _ in self.graph.get(vertex, []) if neighbor not in all_visited)
        print("")

        return list(all_visited)
    
    def get_path(self, v1, v2):
        visited = set()
        path = []

        def dfs_aux(vertice, target):
            visited.add(vertice)
            path.append(vertice)
            if vertice == target:
                return True
            for neighbor, _ in self.graph.get(vertice, []):
                if neighbor not in visited:
                    if dfs_aux(neighbor, target):
                        return True
            path.pop()  # Remove the vertex from path if it doesn't lead to target
            return False

        if dfs_aux(v1, v2):
            return path
        else:
            return [] 

    def min_distance(self, dist, visited):
        min_dist = float('inf')
        min_index = -1

        for v in dist:
            if dist[v] < min_dist and not visited[v]:
                min_dist = dist[v]
                min_index = v

        return min_index

    def print_distances(self, dist):
        print("Vértice \tDistância")
        for i in dist:
            print(f"{i}\t{dist[i]}")
    
    def dijkstra(self, source):
        dist = {v: float('inf') for v in self.graph}
        dist[source] = 0
        visited = [False] * len(self.graph)

        for _ in range(len(self.graph)):
            u = self.min_distance(dist, visited)
            visited[u] = True

            for v, w in self.graph.get(u, []):
                if not visited[v] and dist[u] + w < dist[v]:
                    dist[v] = dist[u] + w

        self.print_distances(dist)

    def min_distance(self, dist, visited):
        min_dist = float('inf')
        min_index = -1

        for v in dist:
            if dist[v] < min_dist and not visited[v]:
                min_dist = dist[v]
                min_index = v

        return min_index
        
    def bellman_ford(self, origem):
        dist = {i: float('inf') for i in self.graph}
        dist[origem] = 0

        for _ in range(len(self.graph) - 1):
            for u in self.graph:
                for v, w in self.graph[u]:
                    if dist[u] != float('inf') and dist[u] + w < dist[v]:
                        dist[v] = dist[u] + w

        self.print_distances(dist)
        
    def print_fwdistances(self, dist):
        for i in range(len(dist)):
            print(f"Vert {i}:")
            for j in range(len(dist[i])):
                if dist[i][j] != float('inf'):
                    print(f"{j}\t{dist[i][j]}")
                else:
                    print(f"{j}\tINF")
        
    def floyd_warshall(self):
        V = len(self.graph)
        dist = [[float('inf')] * V for _ in range(V)]

        for u in range(V):
            dist[u][u] = 0

        for u in self.graph:
            for v, w in self.graph[u]:
                dist[u][v] = w

        for k in self.graph:
            for i in self.graph:
                for j in self.graph:
                    dist[i][j] = min(dist[i][j], dist[i][k] + dist[k][j])

        self.print_fwdistances(dist)
            
    def a_star(self, origem, destino):
        if origem not in self.graph or destino not in self.graph:
            return "Vertices não encontrados no grafo."

        open_set = set([origem])
        closed_set = set()
        dist = {vertice: float('inf') for vertice in self.graph}
        dist[origem] = 0

        while open_set:
            atual = None
            menor_custo = float('inf')

            for vertice in open_set:
                if dist[vertice] < menor_custo:
                    menor_custo = dist[vertice]
                    atual = vertice

            if atual == destino:
                break

            open_set.remove(atual)
            closed_set.add(atual)

            for vizinho, peso in self.graph[atual]:
                if vizinho in closed_set:
                    continue

                novo_custo = dist[atual] + peso
                if novo_custo < dist[vizinho]:
                    dist[vizinho] = novo_custo
                    open_set.add(vizinho)

        self.print_astar_distances(dist, origem, destino)
        
    def print_astar_distances(self, dist, origem, destino):
        print(f"Origem\tDestino\tDistância")
        if dist[destino] != float('inf'):
            print(f"{origem}\t{destino}\t{dist[destino]}")
        else:
            print(f"{origem}\t{destino}\tINF")