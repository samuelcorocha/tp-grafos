class Matrix:
    def __init__(self, n: int, digraph: bool):
        self.n: int = n
        self.graph = [["-"] * n for _ in range(n)]
        self.digraph: bool = digraph

    def get_rate(self, vert):
        if vert >= 0 and vert < self.n:
            soma = 0
            for r in self.n:
                if self.graph[vert][r] != "-":
                    soma += 1
            return soma
        return -1

    def get_graphDegree(self):
        soma = 0
        for l in self.n:
            for c in self.n:
                if (self.graph[l][c] != '-'):
                    soma += 1
        return soma

    def add_edge(self, source, destination, weight=1):
        if source >= 0 and source < self.n and destination >= 0 and destination < self.n:
            self.graph[source][destination] = weight

    def remove_edge(self, source, destination):
        if source >= 0 and source < self.n and destination >= 0 and destination < self.n:
            self.graph[source][destination] = 0

    def __str__(self):
        graph_str = ""
        for row in self.graph:
            graph_str += " ".join(map(str, row)) + "\n"
        return graph_str
