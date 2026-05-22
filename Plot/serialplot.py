import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import collections

# --- CONFIGURAÇÃO ---
PORTA_SERIAL = 'COM5' # Confirme se é a porta correta!
BAUD_RATE = 115200    
MAX_PONTOS = 100      

# Inicia a comunicação serial
try:
    porta = serial.Serial(PORTA_SERIAL, BAUD_RATE, timeout=0.1)
    # Força os pinos lógicos DTR/RTS para avisar a Pico que o PC conectou
    # Isso destrava o "while (!stdio_usb_connected())" do seu C++
    porta.dtr = True
    porta.rts = True
except Exception as e:
    print(f"Erro ao abrir a porta serial: {e}")
    exit()

dados_y = collections.deque(maxlen=MAX_PONTOS)
for i in range(MAX_PONTOS):
    dados_y.append(0)

fig, ax = plt.subplots()
linha, = ax.plot(dados_y, color='blue', linewidth=2)
ax.set_ylim(0, 1500)
ax.set_title("Distância do VL53L0X em Tempo Real")
ax.set_ylabel("Milímetros (mm)")
ax.grid(True)

def atualizar_grafico(frame):
    while porta.in_waiting:
        try:
            linha_texto = porta.readline().decode('utf-8').strip()
            
            # Se a linha estiver vazia, ignora
            if not linha_texto:
                continue
                
            valor = int(linha_texto)
            dados_y.append(valor)
            
        except ValueError:
            # MODO DETETIVE: Se der erro, imprime no terminal o que chegou
            print(f"Lixo ou Texto recebido (Não é número puro): '{linha_texto}'")
            
    linha.set_ydata(dados_y)
    return linha,

# Adicionado 'cache_frame_data=False' para sumir com o UserWarning chato
anim = FuncAnimation(fig, atualizar_grafico, interval=30, blit=True, cache_frame_data=False)

plt.show()
porta.close()