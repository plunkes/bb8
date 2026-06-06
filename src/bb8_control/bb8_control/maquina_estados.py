
from explorador import ExploradorFronteiras
from d_star_lite import DStarLitePersonalizado
import numpy as np

class GerenciadorMissao:
    def __init__(self):
        self.ESTADO_ATUAL = "EXPLORANDO" 
        
        self.explorador = ExploradorFronteiras()
        self.navegador = DStarLitePersonalizado()
        
        self.caminho_atual = []
        self.alvo_atual = None
        
        # --- VARIÁVEIS DOS NOVOS ESTADOS ---
        self.ticks_procurando = 0 
        # Ponto ótimo para não entrar no ponto cego do LIDAR e não colar na câmera
        self.DISTANCIA_COLETA = 0.4 

    def atualizar_estado_e_caminho(self, mapa_2d, posicao_robo, bandeira_detectada, pos_bandeira_grid, distancia_frente):
        if mapa_2d is None or posicao_robo is None:
            return None 
            
        # REGRAS DE TRANSIÇÃO DE ESTADOS
        
        if self.ESTADO_ATUAL == "EXPLORANDO":
            if bandeira_detectada:
                print("[MÁQUINA DE ESTADOS] Bandeira detectada! Mudando para NAVIGANDO_PARA_BANDEIRA.")
                self.ESTADO_ATUAL = "NAVIGANDO_PARA_BANDEIRA"
                
        elif self.ESTADO_ATUAL == "NAVIGANDO_PARA_BANDEIRA":
            if not bandeira_detectada:
                print("[MÁQUINA DE ESTADOS] Perdi a bandeira! Entrando em PROCURANDO_BANDEIRA (Giro 360).")
                self.ESTADO_ATUAL = "PROCURANDO_BANDEIRA"
                self.ticks_procurando = 0
            elif distancia_frente <= self.DISTANCIA_COLETA:
                print("[MÁQUINA DE ESTADOS] Distância ideal atingida! Mudando para POSICIONANDO_PARA_COLETA.")
                self.ESTADO_ATUAL = "POSICIONANDO_PARA_COLETA"
                
        elif self.ESTADO_ATUAL == "PROCURANDO_BANDEIRA":
            if bandeira_detectada:
                print("[MÁQUINA DE ESTADOS] Reencontrei a bandeira no giro! Retomando navegação.")
                self.ESTADO_ATUAL = "NAVIGANDO_PARA_BANDEIRA"
            else:
                self.ticks_procurando += 1
                # Matemática do giro 360º: 12.5s a 0.5 rad/s. A 20Hz = ~251 ticks. Usamos 260 de margem.
                if self.ticks_procurando > 260:
                    print("[MÁQUINA DE ESTADOS] Giro 360º completo. Bandeira não encontrada. Voltando a EXPLORAR.")
                    self.ESTADO_ATUAL = "EXPLORANDO"
                    self.alvo_atual = None

        elif self.ESTADO_ATUAL == "POSICIONANDO_PARA_COLETA":
            if not bandeira_detectada:
                # Se algo passar na frente da bandeira ou o robô esbarrar nela
                self.ESTADO_ATUAL = "PROCURANDO_BANDEIRA"

        
        # AÇÕES DE ACORDO COM O ESTADO
        
        if self.ESTADO_ATUAL == "EXPLORANDO":
            if self.alvo_atual is None or posicao_robo == self.alvo_atual:
                novo_alvo = self.explorador.encontrar_alvo_desconhecido(mapa_2d, posicao_robo)
                if novo_alvo:
                    self.alvo_atual = novo_alvo
                    self.navegador.inicializar_planejamento(mapa_2d, posicao_robo, self.alvo_atual)

        elif self.ESTADO_ATUAL == "NAVIGANDO_PARA_BANDEIRA":
            if pos_bandeira_grid is not None:
                # Se a bandeira for atualizada na visão, ajusta o alvo dinamicamente
                if self.alvo_atual != pos_bandeira_grid:
                    self.alvo_atual = pos_bandeira_grid
                    self.navegador.inicializar_planejamento(mapa_2d, posicao_robo, self.alvo_atual)

        # CALCULAR ROTA COM D* LITE
        if self.ESTADO_ATUAL in ["EXPLORANDO", "NAVIGANDO_PARA_BANDEIRA"] and self.alvo_atual:
            self.navegador.mapa = mapa_2d 
            self.navegador.calcular_caminho_mais_curto()
            self.caminho_atual = self.navegador.extrair_caminho()
            
            if len(self.caminho_atual) > 1:
                return self.caminho_atual[1] 
                
        return None