from collections import deque
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
        visited = []
        all_visited = []
        queue = deque([vertice])

        while queue:
            vertex = queue.popleft()
            if vertex not in visited:
                print(vertex, end="->")
                visited.append(vertex)
                all_visited.append(vertex)
                queue.extend(neighbor for neighbor, _ in self.graph.get(vertex, []) if neighbor not in all_visited)

        # Verifique se todos os vértices foram visitados; se não, encontre um novo ponto de partida.
        while all_visited != set(self.graph.keys()):
            print("")
            start_vertex = next(vertex for vertex in self.graph if vertex not in all_visited)
            queue = deque([start_vertex])
            visited = []
            while queue:
                vertex = queue.popleft()
                if vertex not in visited:
                    print(vertex, end="->")
                    visited.append(vertex)
                    all_visited.append(vertex)
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
    
    def get_gephi_model():
        pass