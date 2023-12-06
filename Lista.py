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

        def dfs_aux(vertice):
            visited.add(vertice)
            path.append(vertice)
            if vertice == v2:
                return True
            for neighbor, _ in self.graph.get(vertice, []):
                    if neighbor not in visited:
                        if dfs_aux(neighbor):
                            return True
            return False
        
        dfs_aux(v1)
        return path

    def print_solution(self, distancias):
        print("Vertice \tDistancia do Vertice de Origem")
        for node in range(self.V):
            print(node, "\t", distancias[node])

    def min_distance(self, distancias, visitados):
        min_dist = float('inf')
        for v in range(self.V):
            if distancias[v] < min_dist and visitados[v] == False:
                min_dist = distancias[v]
                min_index = v
        return min_index

    def dijkstra(self, vertice_origem):
        distancias = [float('inf')] * self.V
        distancias[vertice_origem] = 0
        visitados = [False] * self.V

        for cout in range(self.V):
            u = self.min_distance(distancias, visitados)
            visitados[u] = True

            for v in range(self.V):
                if self.grafo[u][v] > 0 and visitados[v] == False and distancias[v] > distancias[u] + self.grafo[u][v]:
                    distancias[v] = distancias[u] + self.grafo[u][v]

        self.print_solution(distancias)

    def bellman_ford(self, origem):
        distancias = [float('inf')] * len(self.graph)
        predecessores = [None] * len(self.graph)
        distancias[origem] = 0

        for _ in range(len(self.graph) - 1):
            for u in range(len(self.graph)):
                for v, peso in self.graph[u]:
                    if distancias[u] != float('inf') and distancias[u] + peso < distancias[v]:
                        distancias[v] = distancias[u] + peso
                        predecessores[v] = u

        for u in range(len(self.graph)):
            for v, peso in self.graph[u]:
                if distancias[u] != float('inf') and distancias[u] + peso < distancias[v]:
                    return "O grafo contém ciclo de peso negativo"

        self.print_solution(distancias, predecessores)
    
    def floyd_warshall(self):
        n = len(self.graph)
        dist = [[float('inf')]*n for _ in range(n)]
        
        for i in range(n):
            dist[i][i] = 0

        for u in range(n):
            for v, w in self.graph[u]:
                dist[u][v] = w

        for k in range(n):
            for i in range(n):
                for j in range(n):
                    dist[i][j] = min(dist[i][j], dist[i][k] + dist[k][j])

        return dist
    
    def a_star(self, start, goal):
        open_list = []
        g = {v: float('inf') for v in self.graph}
        f = {v: float('inf') for v in self.graph}
        g[start] = 0
        f[start] = h(start, goal)
        heapq.heappush(open_list, (f[start], start))
        parent = {start: None}

        while open_list:
            _, current = heapq.heappop(open_list)

            if current == goal:
                path = []
                while current is not None:
                    path.append(current)
                    current = parent[current]
                return path[::-1], g[goal]  # Retorna o caminho e a distância

            for neighbor, weight in self.graph[current]:
                tentative_g = g[current] + weight
                if tentative_g < g[neighbor]:
                    parent[neighbor] = current
                    g[neighbor] = tentative_g
                    f[neighbor] = g[neighbor] + self.dijksta(neighbor, goal)
                    heapq.heappush(open_list, (f[neighbor], neighbor))

        return None, float('inf')  # Retorna None e infinito se não houver caminho