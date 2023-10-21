class List:
    def __init__(self, n):
        self.graph = {}
        for i in range(n):
            self.graph.append(str(i))

    def add_edge(self, v1, v2, weight=None, directed=False):
        if directed:
            self.graph[v1].append((v2, weight))
        else:
            self.graph[v1].append((v2, weight))
            self.graph[v2].append((v1, weight))
    
    def remove_edge(self, v1, v2, directed=False):
        self.graph[v1].remove(v2)
        if not directed:
            self.graph[v2].remove(v1)
    
    def get_vertex_degree(self, v, directed=False):
        if not directed:
            return len(self.graph[v])
        else:
            entrie_degree = 0

            for i in self.graph.values():
                if v in i:
                    entrie_degree += 1

            output_degree = len(v)

        return {"Entrie": entrie_degree, "Output": output_degree}

    def get_graph_degree(self, directed=False):
        if not directed:
            degree = 0
            for i in self.graph.values():
                degree += len(i)
            return degree
        
        else:
            entrie_degree = 0
            output_degree = 0

            for i in self.graph.values():
                output_degree += len(i)
            
            for i in self.graph.values():
                if i in i:
                    entrie_degree += 1
            
            return {"Entrie": entrie_degree, "Output": output_degree}
        
    def get_vertex_neighborhood(self, v, directed = False):
        neighbors = []
        for i in self.graph[v]:
            neighbors.append(i)

        if not directed:
            return neighbors
        else:
            for i in self.graph.values():
                if not (i == v):
                    if v in self.graph[i]:
                        neighbors.append(i)
    
    def is_conected(self):
        visited = set()
        def dfs(vertex):
            visited.add(vertex)
            for neighbor in self.graph[vertex]:
                if neighbor not in visited and self.graph[neighbor][0] == vertex:
                    dfs(neighbor)
        dfs("1")
        return len(visited) == len(self.graph)

    def is_regular (self, directed = False):
        if not directed: 
            degree = self.get_vertex_degree("1")
            for i in self.graph.values():
                if degree != self.get_vertex_degree(i):
                    return False
            return True
        else:
            entry = False
            out = False
            
            output_degree  = self.get_vertex_degree("1")
            for i in self.graph.values():
                if degree != self.get_vertex_degree(i):
                    out = False
            out = True
            
            quantity = 0
            comparator = 0
            for i in self.graph.values():
                for j in self.graph.values(i):
                    if i in j:
                        quantity += 1
                if i == 1:
                    comparator = quantity
                else:
                    if quantity != comparator:
                        entry = False
                    entry = True
            return (entry and out)

    
    def is_complete(self, directed = False):
        if not directed:
            degree = len(self.graph) - 1
            for i in self.graph.values():
                if degree != self.get_vertex_degree(i):
                    return False
        else:
            degree = len(self.graph) - 1
            for i in self.graph.values():
                if degree != self.get_vertex_degree(i):
                    return False
                if i in self.graph.values():
                    counter += counter
                if counter != degree:
                    return False
        return True
                    