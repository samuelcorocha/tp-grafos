from Lista import List
from Matriz import Matrix


def insertEdge(matrix: Matrix, list: List, directed):
    source = int(input("Vértice de origem: "))
    destination = int(input("Vértice de destino: "))
    weight = int(input("Peso da aresta (ou 1 para aresta não ponderada): "))

    # -----LISTA------
    list = list.add_edge(source, destination, weight, directed)

    # -----MATRIZ-----
    matrix = matrix.add_edge(source, destination, weight)


def removeEdge(matrix: Matrix, list: List, directed):
    source = int(input("Vértice de origem: "))
    destination = int(input("Vértice de destino: "))

    # -----LISTA------
    list = list.add_edge(source, directed)

    # -----MATRIZ-----
    matrix = matrix.remove_edge(source, destination)


def printGraph(matrix: Matrix, list: List):
    print("-----LISTA------")
    print("Lista de Adjacência do Grafo:")
    print(list.graph)

    print("-----MATRIZ-----")
    print("Matriz de Adjacência do Grafo:")
    print(matrix)


def getVertexDegree(matrix: Matrix, list: List, directed):
    vert = int(input("Vértice que deseja consultar: "))
    print("-----LISTA------")
    degree = list.get_vertice_degree(vert, directed)
    if directed:
        print(f"O grau de entrada do vértice {vert} é {degree['Entry']}, e saída é {degree['Output']}")
    else:
        print(f"O grau do vertice {vert} é {degree}")

    print("-----MATRIZ-----")
    if matrix.digraph:
        degree = matrix.get_rate(vert, directed)
        print(
            f"O grau de entrada do vértice {vert} é {degree['entry']}, e saída é {degree['exit']}")
    else:
        degree = matrix.get_rate(vert)
        print(f"O grau do vertice {vert} é {degree['entry']}")


def getGraphDegree(matrix: Matrix, list: List, directed):
    print("-----LISTA------")
    degree = list.get_graph_degree(directed)
    if directed:
        print(f"O grau de entrada do grafo é {degree['Entry']}, e saída é {degree['Output']}")
    else:
        print(f"O grau do grafo é {degree}")

    print("-----MATRIZ-----")
    graphDegree = matrix.get_graphDegree()
    print(f"Grau do grafo (matriz): {graphDegree}")


def find_neighbors(matrix: Matrix, list: List, directed):
    vert = int(input("Qual o vértice que deseja: "))

    print("-----LISTA------")
    neighbors = list.get_vertice_neighborhood(vert, directed)
    if len(neighbors):
        print(f"Os vizinhos são: {neighbors}")
    else:
        print("Não há vizinhos")

    print("-----MATRIZ-----")
    neighbors = matrix.find_neighbors(vert)
    if len(neighbors):
        print(f"Os vizinhos são: {neighbors}")
    else:
        print("Não há vizinhos")


def is_connected(matrix: Matrix, list: List, directed):
    print("-----LISTA------")
    if list.is_connected():
        print("O grafo é conexo.")
    else:
        print("O grafo não é conexo.")

    print("-----MATRIZ-----")
    if not directed:
        if matrix.is_connected():
            print("O grafo é conexo.")
        else:
            print("O grafo não é conexo.")
    else:
        if matrix.is_strongly_connected():
            print("O grafo é fortemente conectado.")
        else:
            print("O grafo não é fortemente conectado.")


def is_regular(matrix: Matrix, list: List, directed):
    print("-----LISTA------")
    if list.is_regular():
        print("O grafo é regular.")
    else:
        print("O grafo não é regular.")

    print("-----MATRIZ-----")
    if matrix.is_regular():
        print("O grafo é regular.")
    else:
        print("O grafo não é regular.")


def is_complete(matrix: Matrix, list: List, directed):
    print("-----LISTA------")
    if list.is_complete():
        print("O grafo é completo.")
    else:
        print("O grafo não é completo.")

    print("-----MATRIZ-----")
    if matrix.is_complete():
        print("O grafo é completo.")
    else:
        print("O grafo não é completo.")

def depth_first_search(matrix: Matrix, list: List):
    vert = int(
        input("Informe o vértice de partida para a busca em profundidade: "))
    print("-----LISTA------")
    search = list.dfs(vert)
    if len(search):
        print(f"Resultado da busca: {search}")
    else:
        print("Nada encontrado")

    print("-----MATRIZ-----")
    visited = [False] * matrix.n
    TD = [0] * matrix.n
    TT = [0] * matrix.n
    parent = [-1] * matrix.n
    t = 0
    print("Realizando busca em profundidade a partir do vértice:", vert)
    #t = matrix.depth_first_search(matrix.n, vert, visited, t)
    search = matrix.dfs(vert, visited, t)
    if len(search):
        print(f"Resultado da busca: {search}")
    else:
        print("Nada encontrado")


def breadth_first_search(matrix: Matrix, list: List):
    vert = int(input("Informe o vértice de partida para a busca em largura: "))
    print("-----LISTA------")
    list.bfs(vert)

    print("-----MATRIZ-----")
    print(f"Vertices visitados: {matrix.bfs(vert)}")


def get_path(matrix: Matrix, list: List, directed):
    source = int(input("Vértice de origem: "))
    destination = int(input("Vértice de destino: "))
    print("-----LISTA------")
    path = list.get_path(source, destination)
    if path != []:
        print("Há o caminho: ")
        print(path)
    else:
        print("Não há caminho")

    print("-----MATRIZ-----")
    if not matrix.get_Path(source, destination):
        print("Não há caminho")

def dijkstra(matrix: Matrix, list: List):
    origin = int(input("Informe o vértice de origem para o algoritmo de Dijkstra: "))
    #print("-----LISTA------")
    #distances_list = list.dijkstra(origin)
    #print(f"Distâncias mínimas a partir do vértice {origin}: {distances_list}")

    print("-----MATRIZ-----")
    distances_matrix = matrix.dijkstra(origin)
    print(f"Distâncias mínimas a partir do vértice {origin}: {distances_matrix}")

def bellman_ford(matrix: Matrix, list: List):
    origin = int(input("Informe o vértice de origem para o algoritmo de Bellman-Ford: "))
    #print("-----LISTA------")
    #try:
        #distances_list = list.bellman_ford(origin)
        #print(f"Distâncias mínimas a partir do vértice {origin}: {distances_list}")
    #except ValueError as e:
     #   print(f"Erro: {e}")

    print("-----MATRIZ-----")
    try:
        distances_matrix = matrix.bellman_ford(origin)
        print(f"Distâncias mínimas a partir do vértice {origin}: {distances_matrix}")
    except ValueError as e:
        print(f"Erro: {e}")

if __name__ == "__main__":
    directed = bool(
        int(input("O grafo será direcionado? (1- Direcionado / 0- Não direcionado): ")))
    n = int(input("Informe o número de vértices: "))
    list = List(n)
    matrix = Matrix(n, directed)

    while True:
        print("\nOpções:")
        print("1. Inserir aresta")
        print("2. Remover aresta")
        print("3. Mostrar grafo")
        print("4. Consultar grau do vértice")
        print("5. Consultar grau do grafo")
        print("6. Consultar vizinhos de um vértice")
        print("7. Verificar se o grafo é conexo")
        print("8. Verificar se o grafo é regular")
        print("9. Verificar se o grafo é completo")
        print("10. Busca em profundidade")
        print("11. Busca em largura")
        print("12. Verificar se há caminho")
        print("13. Dijkstra")
        print("14. Bellman e Ford")

        choice = int(input("Escolha uma opção: "))

        if choice == 1:
            insertEdge(matrix, list, directed)

        elif choice == 2:
            removeEdge(matrix, list, directed)

        elif choice == 3:
            printGraph(matrix, list)

        elif choice == 4:
            getVertexDegree(matrix, list, directed)

        elif choice == 5:
            getGraphDegree(matrix, list, directed)

        elif choice == 6:
            find_neighbors(matrix, list, directed)

        elif choice == 7:
            is_connected(matrix, list, directed)

        elif choice == 8:
            is_regular(matrix, list, directed)

        elif choice == 9:
            is_complete(matrix, list, directed)

        elif choice == 10:
            depth_first_search(matrix, list)

        elif choice == 11:
            breadth_first_search(matrix, list)

        elif choice == 12:
            get_path(matrix, list, directed)
                     
        elif choice == 13:
            dijkstra(matrix, list)

        elif choice == 14:
            bellman_ford(matrix, list)
        else:
            break
