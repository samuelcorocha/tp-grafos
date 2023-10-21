# tp-grafos

Uma biblioteca de grafos feita em Python.

> A biblioteca proporciona a representação do grafo tanto em matriz de adjacência quanto em lista de adjacência.

## Funcionalidades: 
1. Criar o grafo com N vértices. N a ser informado pelo usuário.
   1. Inicialmente o grafo será criado vazio.
2. Inserir e remover arestas.
    1. Lembre-se que as arestas podem ser direcionadas ou não.
    2. Lembre-se que as arestas podem ser ponderadas.
3. Consultar o grau de um vértice.
4. Consultar o grau do grafo.
5. Consultar os vizinhos de um vértice.
6. Verificar se o grafo é conexo.
7. Verificar se o grafo é regular.
8. Verificar se o grafo é completo.
9. Fazer busca em profundidade.
10. Fazer busca em largura.
11. Verificar a existência de caminho entre dois vértices.
    1. Caso exista, exibir o caminho.
12. Permitir a visualização do grafo na ferramenta Gephi. Os formatos suportados são: GEXF, GDF, GML, GraphML, Pajek NET, GraphViz DOT, CSV, UCINET DL, Tulip TPL, Netdraw VNA, Spreadsheet. A documentação de cada formato pode ser encontrada em: https://gephi.org/users/supported-graph-formats