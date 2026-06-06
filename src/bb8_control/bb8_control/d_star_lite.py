import heapq
import numpy as np

class DStarLitePersonalizado:
    def __init__(self):
        self.fila_prioridade = []
        self.g = {}    # Custo real do objetivo até a célula
        self.rhs = {}  # Estimativa de custo baseada nos vizinhos
        self.km = 0.0  # Modificador para quando o robô anda (ajusta a heurística dinamicamente)
        
        self.inicio = None
        self.objetivo = None
        self.mapa = None
        self.resolucao = 1.0

    def heuristica(self, ponto_a, ponto_b):
        """Calcula a distância Euclidiana entre dois pontos."""
        return np.hypot(ponto_a[0] - ponto_b[0], ponto_a[1] - ponto_b[1])

    def calcular_chave(self, s):
        """Calcula a prioridade [k1, k2] de uma célula s na fila."""
        min_g_rhs = min(self.g.get(s, float('inf')), self.rhs.get(s, float('inf')))
        # k1 = min(g, rhs) + heurística até o início + km
        k1 = min_g_rhs + self.heuristica(self.inicio, s) + self.km
        # k2 = min(g, rhs)
        k2 = min_g_rhs
        return (k1, k2)

    def inicializar_planejamento(self, mapa_2d, inicio, objetivo):
        """Prepara o algoritmo com o mapa inicial."""
        self.mapa = mapa_2d
        self.inicio = inicio
        self.objetivo = objetivo
        
        self.g.clear()
        self.rhs.clear()
        self.fila_prioridade = []
        self.km = 0.0
        
        # No D* Lite, planejamos do objetivo para o início
        self.rhs[self.objetivo] = 0.0
        heapq.heappush(self.fila_prioridade, (self.calcular_chave(self.objetivo), self.objetivo))

    def obter_vizinhos(self, u):
        """Retorna os vizinhos válidos (não obstáculos) de uma célula."""
        vizinhos = []
        movimentos = [(0,1), (1,0), (0,-1), (-1,0), (1,1), (-1,-1), (1,-1), (-1,1)]
        altura, largura = self.mapa.shape
        
        for dx, dy in movimentos:
            nx, ny = u[0] + dx, u[1] + dy
            # Se está dentro do mapa e não é obstáculo fatal (ex: > 50 é parede)
            if 0 <= nx < largura and 0 <= ny < altura:
                if self.mapa[ny, nx] < 50 and self.mapa[ny, nx] != -1: 
                    vizinhos.append((nx, ny))
        return vizinhos

    def custo_movimento(self, u, v):
        """Custo de ir do vizinho u para v. Infinito se tiver obstáculo."""
        if self.mapa[v[1], v[0]] >= 50:
            return float('inf')
        return self.heuristica(u, v)

    def atualizar_vertice(self, u):
        """Atualiza a estimativa RHS de uma célula baseada em seus vizinhos."""
        if u != self.objetivo:
            vizinhos = self.obter_vizinhos(u)
            if vizinhos:
                self.rhs[u] = min(self.custo_movimento(u, viz) + self.g.get(viz, float('inf')) for viz in vizinhos)
            else:
                self.rhs[u] = float('inf')
                
        # Remove u da fila se ele já estiver lá
        self.fila_prioridade = [item for item in self.fila_prioridade if item[1] != u]
        heapq.heapify(self.fila_prioridade)
        
        # Se g != rhs, a célula é inconsistente e precisa ser recalculada
        if self.g.get(u, float('inf')) != self.rhs.get(u, float('inf')):
            heapq.heappush(self.fila_prioridade, (self.calcular_chave(u), u))

    def calcular_caminho_mais_curto(self):
        """O coração do D* Lite. Expande os nós até achar o caminho."""
        while self.fila_prioridade:
            chave_topo, u = self.fila_prioridade[0]
            if chave_topo >= self.calcular_chave(self.inicio) and self.rhs.get(self.inicio, float('inf')) == self.g.get(self.inicio, float('inf')):
                break
            
            heapq.heappop(self.fila_prioridade)
            
            g_u = self.g.get(u, float('inf'))
            rhs_u = self.rhs.get(u, float('inf'))
            
            if g_u > rhs_u: # Célula "Overconsistent" (descobrimos um atalho)
                self.g[u] = rhs_u
                for viz in self.obter_vizinhos(u):
                    self.atualizar_vertice(viz)
            else: # Célula "Underconsistent" (um obstáculo apareceu)
                self.g[u] = float('inf')
                self.atualizar_vertice(u)
                for viz in self.obter_vizinhos(u):
                    self.atualizar_vertice(viz)

    def extrair_caminho(self):
        """Após calcular, anda do Início até o Objetivo escolhendo o menor custo."""
        caminho = [self.inicio]
        atual = self.inicio
        
        while atual != self.objetivo:
            vizinhos = self.obter_vizinhos(atual)
            if not vizinhos:
                return [] # Sem saída!
            
            # Escolhe o vizinho que tem o menor (custo_movimento + g)
            melhor_vizinho = min(vizinhos, key=lambda viz: self.custo_movimento(atual, viz) + self.g.get(viz, float('inf')))
            
            # Prevenção de loop infinito se o custo for infinito
            if self.g.get(melhor_vizinho, float('inf')) == float('inf'):
                break
                
            caminho.append(melhor_vizinho)
            atual = melhor_vizinho
            
        return caminho


# ==============================================================================
# --- CÓDIGO DE BACKUP: USO DA BIBLIOTECA (CASO A IMPLEMENTAÇÃO ACIMA FALHE) ---
# ==============================================================================
"""
Para usar o plano de emergência, você precisará instalar a biblioteca:
pip install dstar-lite

E então, dentro do seu arquivo controle_robo.py, em vez de chamar a classe acima, 
você usaria o seguinte código comentado:

from dstar_lite import DStarLite

# No setup do seu robo:
# self.dstar = DStarLite(mapa_numpy, ponto_inicio, ponto_objetivo)

# Para calcular a rota:
# caminho_recalculado = self.dstar.replan()

# Se o SLAM detectar um obstáculo novo:
# self.dstar.update_obstacle(coordenada_x, coordenada_y)
# caminho_recalculado = self.dstar.replan()
"""
