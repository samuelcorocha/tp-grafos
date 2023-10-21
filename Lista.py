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

# 
# graph = {
# 	"1" = ["3" : 2]
# 	"2" = ["1" : 2]
# }            