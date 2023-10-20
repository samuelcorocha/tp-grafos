from Lista import List
from Matriz import Matrix


def insertEdge(matrix: Matrix):
    source = int(input("Vértice de origem: "))
    destination = int(input("Vértice de destino: "))
    weight = int(
        input("Peso da aresta (ou 1 para aresta não ponderada): "))
    matrix.add_edge(source, destination, weight)


def removeEdge(matrix: Matrix):
    source = int(input("Vértice de origem: "))
    destination = int(input("Vértice de destino: "))
    matrix.remove_edge(source, destination)


def printGraph(matrix: Matrix, list: List):
    print("Matrix de Adjacência do Grafo:")
    print(matrix)
    print(list)


def getVertexDegree(matrix: Matrix, list: List):
    vert = int(input("Vértice que deseja consultar: "))
    degree = matrix.get_rate(vert)
    print("O grau do vertice ${vert} é ${degree}")

def getGraphDegree(matrix: Matrix):
    graphDegree = matrix.get_graphDegree()
    print("Grau do grafo (matriz): ${graphDegree}")


if __name__ == "__main__":
    digraph = bool(input("O grafo será direcionado? ( 1- Direcionado / 0- Não direcionado)"))
    n = int(input("Informe o número de vértices: "))
    list = List(n)
    matrix = Matrix(n, digraph)

    while True:
        print("\nOpções:")
        print("1. Inserir aresta")
        print("2. Remover aresta")
        print("3. Mostrar grafo")
        print("4. Consultar grau do vertice")
        print("5. Consultar grau do grafo")

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
        else:
            break
