class Matrix:
    def __init__(self, n, digraph):
        self.vertices = n
        self.digraph = digraph
        self.graph = [["-"] * n for _ in range(n)]

    def add_edge(self, source, destination, weight=None):
        if source >= 0 and source < self.vertices and destination >= 0 and destination < self.vertices:
            self.graph[source][destination] = weight
            if not self.digraph:
                self.graph[destination][source] = weight

    def remove_edge(self, source, destination):
        if source >= 0 and source < self.vertices and destination >= 0 and destination < self.vertices:
            self.graph[source][destination] = "-"
            if not self.digraph:
                self.graph[destination][source] = "-"


def export_undirected_graph(matriz: Matrix, file_name):
    with open(file_name, "w") as f:
        f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        f.write('<gexf xmlns="http://www.gexf.net/1.2draft" version="1.2">\n')
        f.write('  <graph defaultedgetype="undirected">\n')

        f.write('    <nodes>\n')
        for i in range(matriz.vertices):
            f.write(f'      <node id="{i}" label="Node {i}" />\n')
        f.write('    </nodes>\n')

        f.write('    <edges>\n')
        for i in range(matriz.vertices):
            for j in range(i + 1, matriz.vertices):
                if matriz.graph[i][j] != "-":
                    if matriz.graph[i][j] is None:
                        f.write(f'      <edge source="{i}" target="{j}"/>\n')
                    # Caso não seja ponderada verifica se é uma conexão bilateral
                    else:
                        f.write(f'      <edge source="{i}" target="{j}" weight="{matriz.graph[i][j]}"/>\n')
        f.write('    </edges>\n')

        f.write('  </graph>\n')
        f.write('</gexf>\n')


def export_directed_graph(matriz: Matrix, file_name):
    with open(file_name, "w") as f:
        f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        f.write('<gexf xmlns="http://www.gexf.net/1.2draft" version="1.2">\n')
        f.write('  <graph defaultedgetype="directed">\n')

        f.write('    <nodes>\n')
        for i in range(matriz.vertices):
            f.write(f'      <node id="{i}" label="Node {i}" />\n')
        f.write('    </nodes>\n')

        f.write('    <edges>\n')
        for i in range(matriz.vertices):
            for j in range(matriz.vertices):
                if matriz.graph[i][j] != "-":
                    if matriz.graph[i][j] is None:
                        f.write(f'      <edge source="{i}" target="{j}" type="directed"/>\n')
                    else:
                        f.write(f'      <edge source="{i}" target="{j}" type="directed" weight="{matriz.graph[i][j]}"/>\n')

        f.write('    </edges>\n')

        f.write('  </graph>\n')
        f.write('</gexf>\n')


if __name__ == "__main__":

    matriz = Matrix(3, True)
    matriz.add_edge(0, 1)
    matriz.add_edge(2, 1, 10)
    print("Matriz:")
    for i in range(matriz.vertices):
        print(matriz.graph[i])
    if (matriz.digraph):
        export_directed_graph(matriz, "directed_graph.gexf")
    else:
        export_undirected_graph(matriz, "undirected_graph.gexf")
