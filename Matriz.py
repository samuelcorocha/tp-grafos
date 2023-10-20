class Matrix:
    def __init__(self, n: int, digraph: bool):
        self.n: int = n
        self.graph = [["-"] * n for _ in range(n)]
        self.digraph: bool = digraph

    def get_rate(self, vert):
        if vert >= 0 and vert < self.n:
            soma = 0
            for r in range(self.n):  # Corrigindo o loop range
                if self.graph[vert][r] != "-":
                    soma += 1
            return soma

    def get_graphDegree(self):
        soma = 0
        for l in range(self.n):  # Corrigindo o loop range
            for c in range(self.n):  # Corrigindo o loop range
                if (self.graph[l][c] != '-'):
                    soma += 1
        return soma

    def add_edge(self, source, destination, weight=1):
        if source >= 0 and source < self.n and destination >= 0 and destination < self.n:
            self.graph[source][destination] = weight
        if not self.digraph:
            self.graph[destination][source] = weight

    def remove_edge(self, source, destination):
        if source >= 0 and source < self.n and destination >= 0 and destination < self.n:
            self.graph[source][destination] = "-"
        if not self.digraph:
            self.graph[destination][source] = "-"

    def find_neighbors(self, vert):    
        neighbors = []

        for i in range(self.n):
            if self.graph[vert][i] != "-":
                neighbors.append(i)
        return neighbors

    def get_neighbors(self, vert):
        if vert >= 0 and vert < self.n:
            neighbors = []
            for r in range(self.n):
                if self.graph[vert][r] != '-':
                    neighbors.append(r)
            return neighbors

    def is_connected(self):
        # Esta função verifica se o grafo é conexo (para grafos não direcionados).
        # Ela utiliza uma busca em profundidade (DFS) para explorar o grafo.
        visited = [False] * self.n
        self.dfs(0, visited)

        return all(visited)

    def dfs(self, vert, visited):
        # Essa função é uma implementação da busca em profundidade (DFS).
        # Ela é usada para explorar o grafo a partir de um vértice específico.
        visited[vert] = True
        # Marca o vértice atual como visitado, definindo seu estado como 'True' na lista 'visited'.
        for neighbor in self.get_neighbors(vert):
            # Altera sobre os vizinhos do vértice 'vert'.
            if not visited[neighbor]:
                # Se o vizinho não foi visitado (estado 'False' na lista 'visited'), realiza a DFS recursivamente nele.
                self.dfs(neighbor, visited)

    def is_strongly_connected(self):
        # Esta função verifica se o grafo é fortemente conectado (para grafos direcionados).
        # Ela utiliza duas etapas de busca em profundidade (DFS) para explorar o grafo e verificar a conectividade.
        for vert in range(self.n):
            # Inicializa uma lista 'visited' com 'False' para todos os vértices.
            visited = [False] * self.n
            # Realiza uma busca em profundidade (DFS) a partir do vértice atual.
            self.dfs(vert, visited)
            if not all(visited):
                # Se, após a DFS, algum vértice não foi visitado, o grafo não é fortemente conectado.
                # Inverta o grafo (troque as direções das arestas)
                reversed_test = [[self.graph[j][i]
                # Cria uma versão invertida do grafo, trocando as direções das arestas.
                                   for j in range(self.n)] for i in range(self.n)]

        reversed_graph = Matrix(self.n, self.digraph)
        reversed_graph.graph = reversed_test
        for vert in range(self.n):
            visited = [False] * self.n
            # Inicializa uma lista 'visited' com 'False' para todos os vértices.
            reversed_graph.dfs(vert, visited)
            # Realiza uma busca em profundidade (DFS) no grafo invertido.
            if not all(visited):
                # Se, após a segunda DFS, algum vértice não foi visitado, o grafo não é fortemente conectado.
                return False

        return True

    def __str__(self):
        graph_str = ""
        for row in self.graph:
            graph_str += " ".join(map(str, row)) + "\n"
        return graph_str
