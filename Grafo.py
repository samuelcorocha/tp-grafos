from Lista import List
from Matriz import Matrix

def insertEdge(matrix: Matrix, list:List):
    source = int(input("Vértice de origem: "))
    destination = int(input("Vértice de destino: "))
    weight = int(input("Peso da aresta (ou 1 para aresta não ponderada): "))

    print("-----LISTA------")
    list = list.add_edge(source, destination, weight)

    print("-----MATRIZ-----")
    matrix = matrix.add_edge(source, destination, weight)

def removeEdge(matrix: Matrix):
    source = int(input("Vértice de origem: "))
    destination = int(input("Vértice de destino: "))

    print("-----LISTA------")

    print("-----MATRIZ-----")
    matrix = matrix.remove_edge(source, destination)

def printGraph(matrix: Matrix, list: List):
    print("-----LISTA------")
    print("Lista de Adjacência do Grafo:")
    print(list)

    print("-----MATRIZ-----")
    print("Matriz de Adjacência do Grafo:")
    print(matrix)

def getVertexDegree(matrix: Matrix, list: List):
    vert = int(input("Vértice que deseja consultar: "))
    print("-----LISTA------")

    print("-----MATRIZ-----")
    if matrix.digraph:
        degree = matrix.get_rate(vert)
        print(
            f"O grau de entrada do vértice {vert} é {degree['entry']}, e saída é {degree['exit']}")
    else:
        degree = matrix.get_rate(vert)
        print(f"O grau do vertice {vert} é {degree['entry']}")

def getGraphDegree(matrix: Matrix):
    print("-----LISTA------")

    print("-----MATRIZ-----")
    graphDegree = matrix.get_graphDegree()
    print(f"Grau do grafo (matriz): {graphDegree}")

def find_neighbors(matrix: Matrix, vert):
    print("-----LISTA------")

    print("-----MATRIZ-----")
    neighbors = matrix.find_neighbors(vert)
    if len(neighbors):
        print(f"Os vizinhos são: {neighbors}")
    else:
        print("Não há vizinhos")

def is_connected(matrix: Matrix, list:List, connected=False):
    print("-----LISTA------")

    print("-----MATRIZ-----")
    if not connected:
        if matrix.is_connected():
            print("O grafo é conexo.")
        else:
            print("O grafo não é conexo.")
    else:
        if matrix.is_strongly_connected():
            print("O grafo é fortemente conectado.")
        else:
            print("O grafo não é fortemente conectado.")

def is_regular(matrix: Matrix, list: List):
    print("-----LISTA------")

    print("-----MATRIZ-----")
    if matrix.is_regular():
        print("O grafo é regular.")
    else:
        print("O grafo não é regular.")

def is_complete():
    print("-----LISTA------")

    print("-----MATRIZ-----")

def depth_first_search(self, vert, visited, t):
    print("-----LISTA------")

    print("-----MATRIZ-----")
    search = matrix.dfs
    if len(search):
        print(f"Resultado da busca: {search}")
    else:
        print("Nada encontrado")

def breadth_first_search():
    print("-----LISTA------")

    print("-----MATRIZ-----")
    search = matrix.bfs
    if len(search):
        print(f"Resultado da busca: {search}")
    else:
        print("Nada encontrado")

def get_path():
    print("-----LISTA------")

    print("-----MATRIZ-----")

if __name__ == "__main__":
    directed = bool(input("O grafo será direcionado? (1- Direcionado / 0- Não direcionado): "))
    n = int(input("Informe o número de vértices: "))
    list = List(n, directed)
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

        choice = int(input("Escolha uma opção: "))

        if choice == 1:
            insertEdge(matrix)

        elif choice == 2:
            removeEdge(matrix)

        elif choice == 3:
            printGraph(matrix, list)

        elif choice == 4:
            getVertexDegree(matrix, list)

        elif choice == 5:
            getGraphDegree(matrix)

        elif choice == 6:
            vert = int(input("Qual o vértice que deseja: "))
            find_neighbors(matrix, vert)

        elif choice == 7:
            is_connected(matrix)

        elif choice == 8:
            is_regular(matrix, list)

        elif choice == 9 and directed:
            is_connected(matrix)

        elif choice == 10:
            vert = int(input("Informe o vértice de partida para a busca em profundidade: "))
            visited = [False] * matrix.n
            TD = [0] * matrix.n
            TT = [0] * matrix.n
            parent = [-1] * matrix.n
            t = 0
            print("Realizando busca em profundidade a partir do vértice:", vert)
            t = matrix.depth_first_search(matrix.n, vert, visited, t)

        elif choice == 11:
            vert = int(input("Informe o vértice de partida para a busca em largura: "))
            print(f"Vertices visitados: {matrix.bfs(vert)}")

        elif choice == 2:
            source = int(input("Vértice de origem: "))
            destination = int(input("Vértice de destino: "))
            if not matrix.get_Path(source, destination):
                print("Não há caminho")