from Lista import List
from Matriz import Matrix
import random


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
    list = list.remove_edge(source, directed)

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
        print(f"O grau de entrada do vértice {vert} é {degree['entry']}, e saída é {degree['exit']}")
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
    vert = int(input("Informe o vértice de partida para a busca em profundidade: "))
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
    # t = matrix.depth_first_search(matrix.n, vert, visited, t)
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
    source = int(input("Vértice de origem: "))
    print("-----LISTA------")
    dist = list.dijkstra(source)
    print(dist)
    print("-----MATRIZ-----")
    dist = matrix.dijkstra(source)
    print(dist)


def bellman_ford(matrix: Matrix, list: List):
    source = int(input("Vértice de origem: "))
    print("-----LISTA------")
    resultado = list.bellman_ford(source)
    if isinstance(resultado, str):
        print(resultado)  # Se houver um ciclo de peso negativo
    else:
        distancias, predecessores = resultado
        print("Distâncias a partir do vértice de origem:", distancias)
        print("Predecessores:", predecessores)
    print("-----MATRIZ-----")
    dist = matrix.bellman_ford(source)
    print(dist)


def floyd_warshall(matrix: Matrix, list: List):
    print("-----LISTA------")
    resultado = list.floyd_warshall()
    print(resultado)
    print("-----MATRIZ-----")
    matrix.floyd_Warshall()


def a_star(matrix: Matrix, list: List):
    start_state = int(input("Vértice de partida: "))
    goal_state = int(input("Vértice de destino: "))
    print("-----LISTA------")
    path = list.a_star(start_state, goal_state)
    print("Caminho encontrado:", path)
    print("-----MATRIZ-----")
    path = matrix.astar(start_state, goal_state)
    print("Caminho encontrado:", path)


def autofill(matrix: Matrix, list: List):
    for i in range(len(list.graph)):
        for j in range(len(list.graph)):
            if random.random() < 0.3:  # Ajuste este valor para controlar a densidade do grafo
                weight = random.randint(1, 10)
                list.add_edge(i, j, weight, directed)
    for i in range(0, matrix.n):
        for j in range(0, matrix.n):
            if random.random() < 0.3:  # Ajuste este valor para controlar a densidade do grafo
                weight = random.randint(1, 10)
                matrix.add_edge(i, j, weight)


if __name__ == "__main__":
    directed = bool(int(input("O grafo será direcionado? (1- Direcionado / 0- Não direcionado): ")))
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
        print("13. Algoritmo Dijkstra")
        print("14. Algoritmo Bellman e Ford")
        print("15. Algoritmo Floyd Warshall")
        print("16. Algoritmo A*")
        print("17. Preencher automaticamente")

        choice = int(input("Escolha uma opção: "))

        match choice:
            case 1:
                insertEdge(matrix, list, directed)
            case 2:
                removeEdge(matrix, list, directed)
            case 3:
                printGraph(matrix, list)
            case 4:
                getVertexDegree(matrix, list, directed)
            case 5:
                getGraphDegree(matrix, list, directed)
            case 6:
                find_neighbors(matrix, list, directed)
            case 7:
                is_connected(matrix, list, directed)
            case 8:
                is_regular(matrix, list, directed)
            case 9:
                is_complete(matrix, list, directed)
            case 10:
                depth_first_search(matrix, list)
            case 11:
                breadth_first_search(matrix, list)
            case 12:
                get_path(matrix, list, directed)
            case 13:
                dijkstra(matrix, list)
            case 14:
                bellman_ford(matrix, list)
            case 15:
                floyd_warshall(matrix, list)
            case 16:
                a_star(matrix, list)
            case 17:
                autofill(matrix, list)
