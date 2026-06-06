import numpy as np

class ExploradorFronteiras:
    def __init__(self):
        # Valor que o ROS usa para espaço desconhecido no OccupancyGrid
        self.VALOR_DESCONHECIDO = -1
        # Valor para espaço livre onde o robô pode andar com segurança
        self.VALOR_LIVRE = 0
        # Limite de segurança para considerar como obstáculo/parede
        self.LIMITE_OBSTACULO = 50 

    def encontrar_alvo_desconhecido(self, mapa_2d, posicao_robo):
        """
        Varre a matriz do mapa para encontrar todas as fronteiras (células livres
        que são vizinhas de células desconhecidas) e retorna a mais próxima do robô.
        
        Parâmetros:
            mapa_2d (numpy.ndarray): O mapa do SLAM convertido em matriz 2D.
            posicao_robo (tupla): A coordenada atual (x, y) do robô no grid do mapa.
            
        Retorno:
            tupla (x, y): A coordenada da fronteira mais próxima para o robô explorar.
            None: Se o mapa inteiro já estiver explorado.
        """
        altura, largura = mapa_2d.shape
        x_robo, y_robo = posicao_robo
        
        fronteiras_encontradas = []

        # Usamos range(1, ...) para evitar checar as bordas extremas da matriz e dar erro de index
        for y in range(1, altura - 1):
            for x in range(1, largura - 1):
                
                # 1. O robô só pode ir para um lugar que ele sabe que está livre
                if mapa_2d[y, x] == self.VALOR_LIVRE:
                    
                    # 2. Pega os 4 vizinhos em formato de cruz (cima, baixo, esquerda, direita)
                    vizinhos = [
                        mapa_2d[y-1, x], # Cima
                        mapa_2d[y+1, x], # Baixo
                        mapa_2d[y, x-1], # Esquerda
                        mapa_2d[y, x+1]  # Direita
                    ]
                    
                    # 3. Se pelo menos um dos vizinhos é desconhecido, achamos a borda da descoberta!
                    if self.VALOR_DESCONHECIDO in vizinhos:
                        fronteiras_encontradas.append((x, y))

        # Se a lista estiver vazia, significa que não há mais nenhum -1 acessível no mapa.
        if not fronteiras_encontradas:
            return None 

        # --- A ATRAÇÃO PELO DESCONHECIDO ---
        # Entre todas as fronteiras do mapa, queremos a que está mais perto do robô
        # para economizar bateria e tempo. Usamos a hipotenusa (distância reta) para calcular.
        fronteira_mais_proxima = min(
            fronteiras_encontradas, 
            key=lambda f: np.hypot(f[0] - x_robo, f[1] - y_robo)
        )

        return fronteira_mais_proxima