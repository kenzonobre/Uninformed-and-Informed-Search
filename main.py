import pandas as pd
import math
from collections import deque
import heapq

class Vertice:
    def __init__(self, lon, lat, local):
        self.lon = lon
        self.lat = lat
        self.local = local

    def __str__(self):
        return self.local

class Grafo:
    def __init__(self):
        self.vertices = {}
        self.arestas = {}

    def adicionar_vertice(self, vertice):
        if vertice.local not in self.vertices:
            self.vertices[vertice.local] = vertice
            self.arestas[vertice] = []

    def adicionar_aresta(self, vertice1, vertice2):
        self.arestas[vertice1].append(vertice2)

    def __str__(self):
        return f"Grafo com {len(self.vertices)} vertices e {len(self.arestas)} arestas"
    

def distancia_euclidiana(vertice1, vertice2):
  
    lat1 = (3.14 * vertice1.lat) / 180.0
    lon1 = (3.14 * vertice1.lon) / 180.0
    lat2 = (3.14 * vertice2.lat) / 180.0
    lon2 = (3.14 * vertice2.lon) / 180.0

    ab = math.sin(lat1) * math.sin(lat2) + math.cos(lat1) * math.cos(lat2) * math.cos(abs(lon2 - lon1))
    distancia = math.acos(ab) * 6371000

    return distancia

def distancia_manhattan(vertice1, vertice2):

    lat1, lon1 = vertice1.lat, vertice1.lon
    lat2, lon2 = vertice2.lat, vertice2.lon

    vertice3 = Vertice(lon2, lat1, "")

    delta_lon = distancia_euclidiana(vertice1, vertice3)
    delta_lat = distancia_euclidiana(vertice3, vertice2)

    distancia = delta_lon + delta_lat

    return distancia


def BFS(grafo, inicio, destino):
    visitados = set()
    fila = deque()

    vertice_inicio = grafo.vertices[inicio]
    vertice_destino = grafo.vertices[destino]
    
    # Inicializa a busca a partir do vértice de início
    fila.append(vertice_inicio)
    visitados.add(vertice_inicio)

    # Dicionário para armazenar o caminho
    caminho = {vertice_inicio: None}
    distancia = {vertice_inicio: 0}

    while fila:
        vertice_atual = fila.popleft()

        if vertice_atual == vertice_destino:
            # Encontrou o destino, reconstrua o caminho
            caminho_reverso = [vertice_destino.local]
            distancia_final = distancia[vertice_destino]

            while vertice_destino != vertice_inicio:
                vertice_destino = caminho[vertice_destino]
                caminho_reverso.append(vertice_destino.local)

            caminho_final = list(reversed(caminho_reverso))

            return caminho_final, distancia_final

        for vertice_vizinho in grafo.arestas[vertice_atual]:
            if vertice_vizinho not in visitados:
                fila.append(vertice_vizinho)
                visitados.add(vertice_vizinho)
                caminho[vertice_vizinho] = vertice_atual
                distancia[vertice_vizinho] = distancia[vertice_atual] + distancia_manhattan(vertice_atual, vertice_vizinho)

    return None  # Se não encontrou um caminho

def heuristica(vertice_atual, vertice_destino):
    return distancia_euclidiana(vertice_atual, vertice_destino)

def a_estrela(grafo, inicio, destino):
    visitados = set()
    fila = []  # Usaremos uma fila de prioridade (heapq)

    vertice_inicio = grafo.vertices[inicio]
    vertice_destino = grafo.vertices[destino]

    heapq.heappush(fila, (0, vertice_inicio))  # Tupla (prioridade, vértice)
    custo_g = {vertice_inicio: 0}
    caminho = {vertice_inicio: None}

    while fila:
        _, vertice_atual = heapq.heappop(fila)

        if vertice_atual == vertice_destino:
            # Encontrou o destino, reconstrua o caminho
            caminho_reverso = [vertice_destino.local]
            custo_final = custo_g[vertice_destino]
            while vertice_destino != vertice_inicio:
                vertice_destino = caminho[vertice_destino]
                caminho_reverso.append(vertice_destino.local)
            caminho_final = list(reversed(caminho_reverso))
            return caminho_final, custo_final
        
        if vertice_atual in visitados:
            continue

        visitados.add(vertice_atual)

        for vertice_vizinho in grafo.arestas[vertice_atual]:
            if vertice_vizinho not in visitados:
                custo_g_vizinho = custo_g[vertice_atual] + distancia_manhattan(vertice_atual, vertice_vizinho)
                if vertice_vizinho not in custo_g or custo_g_vizinho < custo_g[vertice_vizinho]:
                    heapq.heappush(fila, (custo_g_vizinho + heuristica(vertice_vizinho, vertice_destino), vertice_vizinho))
                    custo_g[vertice_vizinho] = custo_g_vizinho
                    caminho[vertice_vizinho] = vertice_atual

    return None  # Se não encontrou um caminho


# Carregando os dados de vertices e arestas a partir dos arquivos CSV
dados_vertices = pd.read_csv('vertices.csv')
dados_arestas = pd.read_csv('edges.csv')

# Criando um grafo
grafo = Grafo()

# Adicionando vertices ao grafo
for _, linha in dados_vertices.iterrows():
    lon, lat, local = float(linha['lon']), float(linha['lat']), str(linha['name'])
    vertice = Vertice(lon, lat, local)
    grafo.adicionar_vertice(vertice)

# Adicionando arestas ao grafo
for _, linha in dados_arestas.iterrows():
    local1, local2 = str(linha['name1']), str(linha['name2'])

    vertice1 = grafo.vertices[local1]
    vertice2 = grafo.vertices[local2]

    grafo.adicionar_aresta(vertice1, vertice2)
    grafo.adicionar_aresta(vertice2, vertice1)



inicio = "Saída da Mat"
destino = "IQSC"

caminho_BFS, distancia_BFS = BFS(grafo, inicio, destino)
caminho_A_estrela, distancia_A_estrela = a_estrela(grafo, inicio, destino)

print(f"Partindo do(a) {inicio} para chegar na(o) {destino}")

print("\nO algoritmo BFS encontrou o caminho : ")
for i in range(len(caminho_BFS)):
    print(caminho_BFS[i], end='')
    if i < len(caminho_BFS) - 1:
        print(" -> ", end='')
    else:
        print(f"\nCom distância {distancia_BFS} metros.")

print("\nO algoritmo A* encontrou o caminho : ")
for i in range(len(caminho_A_estrela)):
    print(caminho_A_estrela[i], end='')
    if i < len(caminho_A_estrela) - 1:
        print(" -> ", end='')
    else:
        print(f"\nCom distância {distancia_A_estrela} metros.")

