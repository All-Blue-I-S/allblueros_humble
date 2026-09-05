import time
import csv
import os
from datetime import datetime
from collections import deque
import board
from adafruit_bme280 import basic as adafruit_bme280

# =================================================================
# CONFIGURAÇÃO DE HARDWARE (I2C)
# =================================================================
# Endereço padrão de módulos genéricos é 0x76. Módulos Adafruit usam 0x77.
I2C_ADDRESS = 0x76 
try:
    i2c = board.I2C()
    bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c, address=I2C_ADDRESS)
except ValueError:
    print(f"Erro Crítico: BME280 não encontrado no barramento I2C (Endereço {hex(I2C_ADDRESS)}).")
    print("Verifique a fiação e o endereço. O sistema não pode iniciar cego.")
    exit(1)

# =================================================================
# PARÂMETROS DO FILTRO TERMODINÂMICO (AJUSTÁVEIS)
# =================================================================
TAXA_ATUALIZACAO_SEG = 0.5   # O RPi lê o sensor a cada 0.5 segundos (2Hz)
JANELA_ANALISE_SEG = 3.0     # Analisa a variação com base nos últimos 3 segundos

# Limiares críticos de disparo (Ajuste após analisar o primeiro log de mergulho)
LIMIAR_SALTO_UMIDADE_PCT = 6.0   # Salto brusco de umidade em 3s (Micro-vazamento)
LIMIAR_SALTO_PRESSAO_HPA = 4.0   # Compressão brusca do ar em 3s (Entrada volumétrica)
LIMIAR_CHOQUE_TERMICO_C = -1.5   # Queda absurda de temperatura em 3s (Água tocou no chip)

# Cálculo do tamanho do buffer circular
TAMANHO_BUFFER = int(JANELA_ANALISE_SEG / TAXA_ATUALIZACAO_SEG)
hist_T = deque(maxlen=TAMANHO_BUFFER)
hist_P = deque(maxlen=TAMANHO_BUFFER)
hist_H = deque(maxlen=TAMANHO_BUFFER)

# =================================================================
# INICIALIZAÇÃO DO LOG SEGURO (CAIXA PRETA)
# =================================================================
agora = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
nome_arquivo_log = f"log_estanqueidade_auv_{agora}.csv"
arquivo_log = open(nome_arquivo_log, mode='a', newline='')
escritor_csv = csv.writer(arquivo_log)

# Escreve o cabeçalho
escritor_csv.writerow([
    "Timestamp", "Tempo_Run_s", 
    "Temp_C", "Pres_hPa", "Umid_pct", 
    "Delta_T", "Delta_P", "Delta_H", "Status"
])
# Força a gravação física no cartão SD imediatamente
arquivo_log.flush()
os.fsync(arquivo_log.fileno())

# =================================================================
# FUNÇÕES DE SEGURANÇA (FAILSAFE)
# =================================================================
def acionar_failsafe_pixhawk(motivo):
    """
    Integração com PyMAVLink ou pinos GPIO.
    Comunica a Pixhawk para abortar a missão e emergir.
    """
    print("\n" + "!" * 50)
    print(">>> COMANDO DE ABORTO ENVIADO PARA A PIXHAWK <<<")
    print(f"Motivo do Failsafe: {motivo}")
    print("!" * 50 + "\n")
    # Exemplo PyMAVLink: master.mav.command_long_send(...)

# =================================================================
# LOOP PRINCIPAL DA MISSÃO
# =================================================================
print(f"Sistema Armado. Gravando dados em: {nome_arquivo_log}")
print("Enchendo buffer de memória termodinâmica (aguarde 3s)...")

tempo_inicio = time.time()
vazamento_latched = False  # Estado travado (uma vez acionado, não volta ao normal sozinho)

while True:
    try:
        # 1. AQUISIÇÃO DOS DADOS BRUTOS
        T_atual = bme280.temperature
        P_atual = bme280.pressure
        H_atual = bme280.relative_humidity
        
        # Valores padrão para o log enquanto o buffer enche
        delta_T, delta_P, delta_H = 0.0, 0.0, 0.0
        status_atual = "NORMAL"

        # 2. LÓGICA DE DETECÇÃO (Só ativa quando o buffer atingir 3 segundos)
        if len(hist_T) == TAMANHO_BUFFER and not vazamento_latched:
            
            delta_T = T_atual - hist_T[0]
            delta_P = P_atual - hist_P[0]
            delta_H = H_atual - hist_H[0]
            
            # --- CONDIÇÕES FÍSICAS DE VAZAMENTO ---
            
            # Gatilho de Umidade (Evaporação repentina)
            falha_umidade = delta_H > LIMIAR_SALTO_UMIDADE_PCT
            
            # Gatilho Termodinâmico (Pressão sobe muito, ou sobe enquanto a temp cai)
            compressao_severa = delta_P > LIMIAR_SALTO_PRESSAO_HPA
            anomalia_pressao = (delta_P > 1.0) and (delta_T <= 0.0)
            falha_pressao = compressao_severa or anomalia_pressao
            
            # Gatilho Térmico (Choque de água fria direto no sensor)
            choque_termico = delta_T < LIMIAR_CHOQUE_TERMICO_C
            
            # --- TOMADA DE DECISÃO ---
            if falha_umidade or falha_pressao or choque_termico:
                vazamento_latched = True
                status_atual = "EMERGENCIA_VAZAMENTO"
                
                motivo = ""
                if falha_umidade: motivo += f"Pico de Umidade (+{delta_H:.1f}%). "
                if falha_pressao: motivo += f"Anomalia de Compressao (+{delta_P:.1f}hPa com dT {delta_T:.1f}C). "
                if choque_termico: motivo += f"Choque termico ({delta_T:.1f}C)."
                
                acionar_failsafe_pixhawk(motivo)
            
        elif vazamento_latched:
            # Mantém o status travado para o log
            status_atual = "EMERGENCIA_LATENTE"

        # 3. GRAVAÇÃO BLINDADA DO LOG
        tempo_decorrido = time.time() - tempo_inicio
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        
        escritor_csv.writerow([
            timestamp, 
            f"{tempo_decorrido:.1f}", 
            f"{T_atual:.2f}", 
            f"{P_atual:.2f}", 
            f"{H_atual:.2f}", 
            f"{delta_T:.2f}", 
            f"{delta_P:.2f}", 
            f"{delta_H:.2f}", 
            status_atual
        ])
        
        # flush() e fsync() garantem que o sistema operacional grave no cartão SD agora
        arquivo_log.flush()
        os.fsync(arquivo_log.fileno())

        # 4. ATUALIZA A JANELA MÓVEL (Descarta leitura velha, insere nova)
        hist_T.append(T_atual)
        hist_P.append(P_atual)
        hist_H.append(H_atual)
        
        # Feedback no terminal (opcional durante o voo, útil em bancada)
        print(f"[{timestamp}] T:{T_atual:5.1f}C | P:{P_atual:6.1f}hPa | H:{H_atual:4.1f}% | Sts: {status_atual}")

        time.sleep(TAXA_ATUALIZACAO_SEG)
        
    except KeyboardInterrupt:
        print(f"\nMonitoramento abortado pelo usuário. Log salvo em: {nome_arquivo_log}")
        arquivo_log.close()
        break
    except Exception as e:
        # Se um ruído corromper um pacote I2C, o programa sobrevive
        print(f"[{datetime.now().strftime('%H:%M:%S')}] Erro I2C ignorado: {e}")
        time.sleep(TAXA_ATUALIZACAO_SEG)



# WITHOUT LOG =================================================================================

import time
from datetime import datetime
from collections import deque
import board
from adafruit_bme280 import basic as adafruit_bme280

# =================================================================
# CONFIGURAÇÃO DE HARDWARE (I2C)
# =================================================================
I2C_ADDRESS = 0x76 
try:
    i2c = board.I2C()
    bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c, address=I2C_ADDRESS)
except ValueError:
    print(f"Erro Crítico: BME280 não encontrado no barramento I2C (Endereço {hex(I2C_ADDRESS)}).")
    exit(1)

# =================================================================
# PARÂMETROS DO FILTRO TERMODINÂMICO
# =================================================================
TAXA_ATUALIZACAO_SEG = 0.5   # 2Hz
JANELA_ANALISE_SEG = 3.0     # Analisa a variação com base nos últimos 3 segundos

LIMIAR_SALTO_UMIDADE_PCT = 6.0   
LIMIAR_SALTO_PRESSAO_HPA = 4.0   
LIMIAR_CHOQUE_TERMICO_C = -1.5   

# Buffers da Janela Móvel
TAMANHO_BUFFER = int(JANELA_ANALISE_SEG / TAXA_ATUALIZACAO_SEG)
hist_T = deque(maxlen=TAMANHO_BUFFER)
hist_P = deque(maxlen=TAMANHO_BUFFER)
hist_H = deque(maxlen=TAMANHO_BUFFER)

# =================================================================
# FUNÇÃO DE EMERGÊNCIA (FAILSAFE)
# =================================================================
def acionar_failsafe_pixhawk(motivo):
    print("\n" + "!" * 50)
    print(">>> COMANDO DE ABORTO ENVIADO PARA A PIXHAWK <<<")
    print(f"Motivo: {motivo}")
    print("!" * 50 + "\n")
    # Aqui entra o comando PyMAVLink para subir o submarino

# =================================================================
# LOOP PRINCIPAL DA MISSÃO
# =================================================================
print("Sistema Armado. Enchendo buffer de memória (aguarde 3s)...")

vazamento_latched = False

while True:
    try:
        # 1. LEITURA FÍSICA
        T_atual = bme280.temperature
        P_atual = bme280.pressure
        H_atual = bme280.relative_humidity
        
        status_atual = "NORMAL"

        # 2. LÓGICA DE DETECÇÃO (Analisa a taxa de variação)
        if len(hist_T) == TAMANHO_BUFFER and not vazamento_latched:
            
            delta_T = T_atual - hist_T[0]
            delta_P = P_atual - hist_P[0]
            delta_H = H_atual - hist_H[0]
            
            # --- CONDIÇÕES DE VAZAMENTO ---
            falha_umidade = delta_H > LIMIAR_SALTO_UMIDADE_PCT
            compressao_severa = delta_P > LIMIAR_SALTO_PRESSAO_HPA
            anomalia_pressao = (delta_P > 1.0) and (delta_T <= 0.0)
            falha_pressao = compressao_severa or anomalia_pressao
            
            choque_termico = delta_T < LIMIAR_CHOQUE_TERMICO_C
            
            # --- TOMADA DE DECISÃO ---
            if falha_umidade or falha_pressao or choque_termico:
                vazamento_latched = True
                status_atual = "EMERGENCIA!"
                
                motivo = ""
                if falha_umidade: motivo += f"Pico de Umidade (+{delta_H:.1f}%). "
                if falha_pressao: motivo += f"Compressao/Anomalia (+{delta_P:.1f}hPa). "
                if choque_termico: motivo += f"Choque termico ({delta_T:.1f}C)."
                
                acionar_failsafe_pixhawk(motivo)
            
        elif vazamento_latched:
            status_atual = "TRAVADO EM EMERGENCIA"

        # 3. ATUALIZA A JANELA MÓVEL (Descarta o mais antigo automaticamente)
        hist_T.append(T_atual)
        hist_P.append(P_atual)
        hist_H.append(H_atual)
        
        # 4. FEEDBACK NO TERMINAL
        agora = datetime.now().strftime("%H:%M:%S")
        print(f"[{agora}] T: {T_atual:5.1f}C | P: {P_atual:6.1f}hPa | H: {H_atual:4.1f}% | Sts: {status_atual}")

        time.sleep(TAXA_ATUALIZACAO_SEG)
        
    except KeyboardInterrupt:
        print("\nMonitoramento encerrado pelo usuário.")
        break
    except Exception as e:
        print(f"Erro I2C: {e}")
        time.sleep(TAXA_ATUALIZACAO_SEG)