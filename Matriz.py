from collections import deque
import heapq


class Matrix:
    def __init__(self, n: int, digraph: bool):
        self.n: int = n
        self.graph = [["-"] * n for _ in range(n)]
        self.digraph: bool = digraph

    def get_rate(self, vert):
        if vert >= 0 and vert < self.n:
            degree = {"entry": 0, "exit": 0}
            for r in range(self.n):  # Corrigindo o loop range
                if self.graph[vert][r] != "-":
                    degree["entry"] += 1
                if self.graph[r][vert] != "-":
                    degree["exit"] += 1
            return degree

    def get_graphDegree(self):
        soma = 0
        for l in range(self.n):  # Corrigindo o loop range
            for c in range(self.n):  # Corrigindo o loop range
                if (self.graph[l][c] != '-'):
                    soma += 1
        return soma

    def add_edge(self, source, destination, weight=1):
        # Adiciona uma aresta ao grafo, conectando o vértice de origem ao vértice de destino com um peso opcional.
        if source >= 0 and source < self.n and destination >= 0 and destination < self.n:
            if self.graph[destination][source] != "-" and not self.digraph:
                return

            # Verifica se os vértices de origem e destino são válidos.
            self.graph[source][destination] = weight
            # Define o valor da matriz de adjacência para representar a conexão entre os vértices com o peso especificado.
            if not self.digraph:
                # Se o grafo não for direcionado, é necessário adicionar a aresta no sentido oposto.
                self.graph[destination][source] = weight

    def remove_edge(self, source, destination):
        # Remove uma aresta do grafo, desconectando o vértice de origem do vértice de destino.
        if source >= 0 and source < self.n and destination >= 0 and destination < self.n:
            # Verifica se os vértices de origem e destino são válidos.
            self.graph[source][destination] = "-"
            # Remove a conexão entre os vértices, representando-a com um valor especial (neste caso, "-").
        if not self.digraph:
            # Se o grafo não for direcionado, é necessário remover a aresta no sentido oposto.
            self.graph[destination][source] = "-"

    def find_neighbors(self, vert):
        # Obtém a lista de vizinhos de um vértice específico no grafo.
        neighbors = []
        # Inicializa uma lista vazia para armazenar os vizinhos do vértice.

        for i in range(self.n):
            # Itera através de todos os vértices do grafo (de 0 a self.n - 1).
            if self.graph[vert][i] != "-":
                # Verifica se existe uma aresta (ou conexão) entre o vértice 'vert' e o vértice 'i'.
                # Se houver uma aresta, adicione 'i' à lista de vizinhos.
                neighbors.append(i)
        return neighbors

    def get_neighbors(self, vert):
        # Obtém a lista de vizinhos de um vértice específico no grafo.
        if vert >= 0 and vert < self.n:
            # Verifica se o vértice fornecido está dentro dos limites válidos (de 0 a self.n - 1).
            neighbors = []
            # Inicializa uma lista vazia para armazenar os vizinhos do vértice.
            for r in range(self.n):
                # Itera através de todos os vértices do grafo (de 0 a self.n - 1).
                if self.graph[vert][r] != '-':
                    # Verifica se existe uma aresta (ou conexão) entre o vértice 'vert' e o vértice 'r'.
                    # Se houver uma aresta, adicione 'r' à lista de vizinhos.
                    neighbors.append(r)
            return neighbors

    def is_connected(self):
        # Verifica se o grafo é conexo (para grafos não direcionados).
        visited = [False] * self.n

        def dfs(vert):
            visited[vert] = True
            for neighbor in self.get_neighbors(vert):
                if not visited[neighbor]:
                    dfs(neighbor)

        dfs(0)  # Executa a DFS a partir do vértice 0

        # Verifica se todos os vértices foram visitados após a DFS
        return all(visited)

    def is_strongly_connected(self):
        # Verifica se o grafo é fortemente conectado (para grafos direcionados).
        for vert in range(self.n):
            # Inicializa uma lista 'visited' com 'False' para todos os vértices.
            visited = [False] * self.n
            # Realiza uma busca em profundidade a partir do vértice atual.
            t = self.dfs(vert, visited, 0)
            if not all(visited):
                # Se, após a busca, algum vértice não foi visitado, o grafo não é fortemente conectado.
                # Inverte o grafo (troca as direções das arestas)
                reversed_test = [[self.graph[j][i]
                                  for j in range(self.n)] for i in range(self.n)]
                reversed_graph = Matrix(self.n, self.digraph)
                reversed_graph.graph = reversed_test
                for vert in range(self.n):
                    visited = [False] * self.n
                    # Inicializa uma lista 'visited' com 'False' para todos os vértices.
                    t = reversed_graph.dfs(vert, visited, t)
                    # Realiza uma busca em profundidade (DFS) no grafo invertido.
                    if not all(visited):
                        # Se, após a segunda DFS, algum vértice não foi visitado, o grafo não é fortemente conectado.
                        return False
        return True

    def dfs(self, start, visited):
        visited[start] = True
        print(f"Visitando vértice {start}")

        for vertex in range(self.n):
            weight = self.graph[start][vertex]
            if not visited[vertex] and weight != "-":
                print(f"Explorando aresta de {start} para {vertex} com peso {weight}")
                self.dfs(vertex, visited)

    def depth_first_search(self, start):
        if start < 0 or start >= self.n:
            raise ValueError("O vértice inicial está fora do intervalo válido.")

        visited = [False] * self.n
        print(f"Iniciando busca a partir do vértice {start}")
        self.dfs(start, visited)

    def __str__(self):
        # Inicializa uma string vazia para armazenar a representação do grafo em formato de string.
        graph_str = ""
        # Percorre cada linha na matriz de adjacência (representando os vértices do grafo).
        for row in self.graph:
            # Converte os elementos na linha em formato de string e os une com espaços entre eles.
            # Adiciona uma quebra de linha no final para separar as linhas na representação da matriz.
            graph_str += " ".join(map(str, row)) + "\n"
        # Retorna a string resultante que representa a matriz de adjacência do grafo.
        return graph_str

    def is_regular(self):
        vertices = {}
        for i in range(self.n):
            vertices[i] = 0
        for i in range(self.n):
            for j in range(self.n):
                if self.graph[i][j] != "-" and i != j:
                    vertices[i] += 1
        flag = vertices[0]
        for i in range(self.n):
            if vertices[i] != flag:
                return False
        return True

    def is_complete(self):
        for i in range(self.n):
            for j in range(self.n):
                if i != j and self.graph[i][j] == "-":
                    return False
        return True

    def bfs(self, origin):
        if origin < 0 or origin >= self.n:
            raise ValueError(
                "O vértice inicial está fora do intervalo válido.")

        visited = [False] * self.n
        queue = deque()
        fila = []

        visited[origin] = True
        queue.append(origin)

        while queue:
            current_vertex = queue.popleft()
            fila.append(current_vertex)
            print(f"Visitando vértice {current_vertex}")

            for neighbor in range(self.n):
                if self.graph[current_vertex][neighbor] != "-" and not visited[neighbor]:
                    visited[neighbor] = True
                    queue.append(neighbor)
        return fila

    def buscaCaminho(self, vertice_inicial):
        visitados = []
        stack = [vertice_inicial]

        while stack:
            v = stack.pop()
            visitados.append(v)

            for u in range(len(self.graph)):
                if self.graph[v][u] != "-" and u not in visitados:
                    stack.append(u)

        return visitados

    def get_Path(self, origin, destination):

        visiteds = set()
        pilha = [origin]

        while pilha:
            v = pilha.pop()
            visiteds.add(v)

            if v == destination:
                print(self.buscaCaminho(origin))
                return True

            for u in range(len(self.graph)):
                if self.graph[v][u] != "-" and u not in visiteds:
                    pilha.append(u)

        return False

    def dijkstra(self, origin):
        if origin < 0 or origin >= self.n:
            raise ValueError("O vértice inicial está fora do intervalo válido.")

        # Inicializa as distâncias a partir do vértice de origem como infinito para todos os vértices
        distances = [float('inf')] * self.n
        distances[origin] = 0

        # Inicializa um conjunto para acompanhar os vértices já visitados
        visited = set()

        # Loop para encontrar o caminho mais curto para todos os vértices
        while len(visited) < self.n:
            # Escolhe o vértice não visitado mais próximo
            min_distance = float('inf')
            min_vertex = -1
            for v in range(self.n):
                if distances[v] < min_distance and v not in visited:
                    min_distance = distances[v]
                    min_vertex = v

            # Adiciona o vértice escolhido ao conjunto de visitados
            visited.add(min_vertex)

            # Atualiza as distâncias dos vizinhos do vértice escolhido
            for neighbor in range(self.n):
                weight = self.graph[min_vertex][neighbor]
                if weight != "-" and distances[min_vertex] + int(weight) < distances[neighbor]:
                    distances[neighbor] = distances[min_vertex] + int(weight)

        return distances

    def bellman_ford(self, origin):
        if origin < 0 or origin >= self.n:
            raise ValueError("O vértice inicial está fora do intervalo válido.")

        # Inicializa as distâncias e predecessores
        distances = [float('inf')] * self.n
        distances[origin] = 0
        predecessors = [-1] * self.n

        # Relaxamento das arestas
        for _ in range(self.n - 1):
            for u in range(self.n):
                for v in range(self.n):
                    weight = self.graph[u][v]
                    if weight != "-" and distances[u] != float('inf') and distances[u] + weight < distances[v]:
                        distances[v] = distances[u] + weight
                        predecessors[v] = u

        # Verificação de ciclos negativos
        for u in range(self.n):
            for v in range(self.n):
                weight = self.graph[u][v]
                if weight != "-" and distances[u] != float('inf') and distances[u] + weight < distances[v]:
                    return ["O grafo contém ciclo negativo"]

        return distances, predecessors

    def floyd_Warshall(self):
        for i in range(self.n):
            for j in range(self.n):
                if self.graph[i][j] == "-" and i == j:
                    self.graph[i][j] = 0
        dist = [row[:] for row in self.graph]

        for k in range(self.n):
            for i in range(self.n):
                for j in range(self.n):
                    if dist[i][k] != "-" and dist[k][j] != "-" and (dist[i][j] == "-" or dist[i][k] + dist[k][j] < dist[i][j]):
                        dist[i][j] = dist[i][k] + dist[k][j]
        for i in dist:
            print(i)

    def astar(self, start, goal):
        heap = [(0, start)]
        visited = set()
        while heap:
            (cost, node) = heapq.heappop(heap)
            if node in visited:
                continue

            visited.add(node)

            if node == goal:
                return cost

            for next_node, weight in enumerate(self.graph[node]):
                if weight != "-":
                    heapq.heappush(heap, (cost + int(weight), next_node))

        return float('inf')