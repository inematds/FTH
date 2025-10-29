---
layout: page
title: "Módulo 2.1: Programação com Python e ROS 2"
permalink: /niveis/nivel-2/modulo-1/
---

# Módulo 2.1: Programação com Python e ROS 2

**Duração estimada:** 12-15 horas
**Nível:** Intermediário
**Pré-requisito:** Nível 1 concluído

---

## Objetivos de Aprendizado

Ao final deste módulo, você será capaz de:

- [ ] Programar em Python aplicado à robótica
- [ ] Configurar ambiente de desenvolvimento Python
- [ ] Compreender os conceitos fundamentais de ROS 2
- [ ] Criar e executar nodes ROS 2
- [ ] Publicar e subscrever topics
- [ ] Integrar ROS 2 com simuladores (Webots)
- [ ] Controlar um robô humanoide via código Python

---

## Parte 1: Python para Robótica (4-5h)

### 1.1 Por que Python?

Python é a linguagem dominante em robótica e IA por várias razões:

- **Sintaxe simples:** Fácil de ler e escrever
- **Bibliotecas poderosas:** NumPy, OpenCV, TensorFlow, PyTorch
- **Comunidade ativa:** Muitos exemplos e suporte
- **Integração:** ROS 2, simuladores, frameworks de IA

### 1.2 Instalação do Ambiente Python

#### Windows

```bash
# 1. Baixe Python 3.10+ em python.org
# 2. Durante instalação, marque "Add Python to PATH"
# 3. Verifique instalação
python --version
pip --version

# 4. Instale virtualenv
pip install virtualenv

# 5. Crie ambiente virtual para robótica
cd C:\robotica
python -m venv robo_env

# 6. Ative ambiente virtual
robo_env\Scripts\activate

# 7. Instale bibliotecas essenciais
pip install numpy matplotlib scipy
```

#### Linux/Mac

```bash
# 1. Python já vem instalado, mas instale pip
sudo apt update
sudo apt install python3-pip python3-venv

# 2. Crie ambiente virtual
mkdir -p ~/robotica
cd ~/robotica
python3 -m venv robo_env

# 3. Ative ambiente virtual
source robo_env/bin/activate

# 4. Instale bibliotecas
pip install numpy matplotlib scipy
```

### 1.3 Python Básico - Revisão Rápida

#### Variáveis e Tipos

```python
# Variáveis básicas
velocidade = 0.5  # float - velocidade do robô em m/s
angulo = 90       # int - ângulo em graus
nome_robo = "Bumi"  # string
esta_ativo = True  # boolean

# Listas - coleções ordenadas
posicao = [0.0, 0.0, 0.5]  # [x, y, z]
sensores = ["camera", "lidar", "imu"]

# Dicionários - pares chave-valor
robo_config = {
    "nome": "Bumi",
    "altura": 1.45,
    "sensores": ["camera", "lidar"],
    "ativo": True
}

print(f"Robô {robo_config['nome']} tem altura de {robo_config['altura']}m")
```

#### Funções

```python
def calcular_distancia(x1, y1, x2, y2):
    """
    Calcula distância euclidiana entre dois pontos 2D

    Args:
        x1, y1: Coordenadas do ponto 1
        x2, y2: Coordenadas do ponto 2

    Returns:
        float: Distância entre os pontos
    """
    import math
    dx = x2 - x1
    dy = y2 - y1
    return math.sqrt(dx**2 + dy**2)

# Uso
distancia = calcular_distancia(0, 0, 3, 4)
print(f"Distância: {distancia}m")  # Output: 5.0m
```

#### Classes - Programação Orientada a Objetos

```python
class RoboHumanoide:
    """Classe para representar um robô humanoide"""

    def __init__(self, nome, altura):
        """Construtor - inicializa o robô"""
        self.nome = nome
        self.altura = altura
        self.posicao = [0.0, 0.0, 0.0]  # [x, y, z]
        self.velocidade = 0.0
        self.ativo = False

    def ligar(self):
        """Liga o robô"""
        self.ativo = True
        print(f"{self.nome} foi ligado!")

    def andar(self, velocidade):
        """Define velocidade de caminhada"""
        if not self.ativo:
            print("Erro: Robô está desligado!")
            return

        self.velocidade = velocidade
        print(f"{self.nome} andando a {velocidade}m/s")

    def mover_para(self, x, y, z):
        """Move robô para posição específica"""
        self.posicao = [x, y, z]
        print(f"{self.nome} moveu para posição {self.posicao}")

    def status(self):
        """Retorna status atual do robô"""
        return {
            "nome": self.nome,
            "ativo": self.ativo,
            "posicao": self.posicao,
            "velocidade": self.velocidade
        }

# Uso da classe
bumi = RoboHumanoide("Bumi", 1.45)
bumi.ligar()
bumi.andar(0.5)
bumi.mover_para(1.0, 2.0, 0.0)
print(bumi.status())
```

### 1.4 NumPy - Matemática para Robótica

```python
import numpy as np

# Arrays - estruturas eficientes para cálculos
posicao = np.array([1.0, 2.0, 0.5])  # Posição 3D
velocidade = np.array([0.5, 0.0, 0.0])  # Velocidade em x

# Operações vetoriais
nova_posicao = posicao + velocidade * 1.0  # dt = 1 segundo
print(f"Nova posição: {nova_posicao}")

# Matrizes de rotação
angulo = np.radians(45)  # 45 graus em radianos
matriz_rotacao = np.array([
    [np.cos(angulo), -np.sin(angulo), 0],
    [np.sin(angulo),  np.cos(angulo), 0],
    [0, 0, 1]
])

# Rotacionar vetor
vetor_original = np.array([1, 0, 0])
vetor_rotacionado = matriz_rotacao @ vetor_original  # @ = multiplicação matricial
print(f"Vetor rotacionado: {vetor_rotacionado}")

# Funções úteis
distancias = np.array([0.5, 1.2, 0.8, 2.1])
print(f"Distância mínima: {np.min(distancias)}")
print(f"Distância média: {np.mean(distancias)}")
print(f"Índice do mínimo: {np.argmin(distancias)}")
```

### 1.5 Exercício Prático 1: Simulador de Caminhada

Crie um programa que simula a caminhada do robô:

```python
import numpy as np
import matplotlib.pyplot as plt

class SimuladorCaminhada:
    """Simula caminhada de robô humanoide"""

    def __init__(self):
        self.posicao = np.array([0.0, 0.0])
        self.historico = [self.posicao.copy()]

    def dar_passo(self, direcao, tamanho_passo=0.3):
        """
        Simula um passo

        Args:
            direcao: Ângulo em graus (0=frente, 90=esquerda)
            tamanho_passo: Comprimento do passo em metros
        """
        angulo_rad = np.radians(direcao)
        dx = tamanho_passo * np.cos(angulo_rad)
        dy = tamanho_passo * np.sin(angulo_rad)

        self.posicao += np.array([dx, dy])
        self.historico.append(self.posicao.copy())

    def caminhar_reta(self, num_passos):
        """Caminha em linha reta para frente"""
        for _ in range(num_passos):
            self.dar_passo(0)  # 0 graus = frente

    def girar_e_caminhar(self, angulo, num_passos):
        """Gira e caminha"""
        for _ in range(num_passos):
            self.dar_passo(angulo)

    def plotar_trajetoria(self):
        """Visualiza trajetória do robô"""
        historico = np.array(self.historico)

        plt.figure(figsize=(10, 10))
        plt.plot(historico[:, 0], historico[:, 1], 'b-', linewidth=2, label='Trajetória')
        plt.plot(historico[0, 0], historico[0, 1], 'go', markersize=10, label='Início')
        plt.plot(historico[-1, 0], historico[-1, 1], 'ro', markersize=10, label='Fim')

        plt.xlabel('X (metros)')
        plt.ylabel('Y (metros)')
        plt.title('Trajetória do Robô')
        plt.grid(True)
        plt.legend()
        plt.axis('equal')
        plt.show()

# Teste o simulador
sim = SimuladorCaminhada()
sim.caminhar_reta(10)          # 10 passos para frente
sim.girar_e_caminhar(90, 5)    # Gira 90° e dá 5 passos
sim.girar_e_caminhar(0, 5)     # Volta para frente, 5 passos
sim.plotar_trajetoria()

print(f"Posição final: {sim.posicao}")
```

**Desafio:** Modifique o código para fazer o robô andar em um quadrado perfeito!

---

## Parte 2: ROS 2 - Robot Operating System (5-6h)

### 2.1 O que é ROS 2?

ROS 2 (Robot Operating System 2) é um framework para desenvolvimento de software de robótica. Pense nele como um "sistema nervoso" que conecta todas as partes do robô.

**Principais conceitos:**

```
┌─────────────────────────────────────────────┐
│              ROS 2 SYSTEM                   │
│                                             │
│  ┌───────┐      Topic      ┌───────┐      │
│  │ Node  │ ──────────────> │ Node  │      │
│  │   A   │   /cmd_vel      │   B   │      │
│  └───────┘                 └───────┘      │
│     │                          │           │
│     │                          │           │
│     └─────── Service ──────────┘          │
│              /start_walk                   │
└─────────────────────────────────────────────┘
```

- **Node:** Processo independente (ex: controle de perna, processamento de câmera)
- **Topic:** Canal de comunicação unidirecional (ex: velocidade do robô)
- **Message:** Dados trafegados (ex: velocidade, imagem, posição)
- **Service:** Requisição/resposta síncrona (ex: "ligue o motor")

### 2.2 Instalação ROS 2 Humble

#### Ubuntu 22.04 (recomendado)

```bash
# 1. Setup sources
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe

# 2. Adicionar ROS 2 apt repository
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 3. Instalar ROS 2 Humble
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop python3-colcon-common-extensions

# 4. Setup ambiente
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 5. Verificar instalação
ros2 --help
```

#### Windows (WSL2 ou Docker)

Recomendamos usar WSL2 (Windows Subsystem for Linux) e seguir as instruções Ubuntu acima.

### 2.3 Primeiro Node ROS 2

Crie um workspace ROS 2:

```bash
# Criar workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Criar pacote Python
ros2 pkg create --build-type ament_python meu_robo_py
cd meu_robo_py
```

Edite `meu_robo_py/primeiro_node.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class PrimeiroNode(Node):
    """
    Nosso primeiro ROS 2 Node!
    """

    def __init__(self):
        # Inicializa o node com nome 'primeiro_node'
        super().__init__('primeiro_node')

        # Cria timer que chama callback a cada 1 segundo
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.contador = 0

        self.get_logger().info('Primeiro Node inicializado!')

    def timer_callback(self):
        """Função chamada periodicamente pelo timer"""
        self.contador += 1
        self.get_logger().info(f'Olá do ROS 2! Contador: {self.contador}')

def main(args=None):
    # Inicializa ROS 2
    rclpy.init(args=args)

    # Cria instância do node
    node = PrimeiroNode()

    # Mantém node rodando
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Configurar `setup.py`:

```python
entry_points={
    'console_scripts': [
        'primeiro_node = meu_robo_py.primeiro_node:main',
    ],
},
```

Build e execute:

```bash
cd ~/ros2_ws
colcon build --packages-select meu_robo_py
source install/setup.bash
ros2 run meu_robo_py primeiro_node
```

### 2.4 Publisher e Subscriber

#### Publisher - Publicando velocidade do robô

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RoboPublisher(Node):
    """
    Publica comandos de velocidade para o robô
    """

    def __init__(self):
        super().__init__('robo_publisher')

        # Cria publisher no topic /cmd_vel
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer para publicar a cada 0.5 segundos
        self.timer = self.create_timer(0.5, self.publicar_velocidade)

        self.velocidade_linear = 0.0
        self.velocidade_angular = 0.0

        self.get_logger().info('Publisher de velocidade iniciado!')

    def publicar_velocidade(self):
        """Publica mensagem de velocidade"""
        msg = Twist()

        # Define velocidades
        msg.linear.x = self.velocidade_linear  # Frente/trás
        msg.linear.y = 0.0                     # Esquerda/direita (robô humanoide não usa)
        msg.linear.z = 0.0                     # Cima/baixo

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.velocidade_angular  # Rotação

        self.publisher.publish(msg)
        self.get_logger().info(f'Publicado: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}')

        # Varia velocidade (exemplo)
        self.velocidade_linear += 0.1
        if self.velocidade_linear > 1.0:
            self.velocidade_linear = 0.0

def main(args=None):
    rclpy.init(args=args)
    node = RoboPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Subscriber - Lendo dados de sensor

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class SensorSubscriber(Node):
    """
    Subscreve dados de LIDAR e detecta obstáculos
    """

    def __init__(self):
        super().__init__('sensor_subscriber')

        # Subscreve ao topic /scan (LIDAR)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.callback_lidar,
            10
        )

        self.get_logger().info('Subscriber de LIDAR iniciado!')

    def callback_lidar(self, msg):
        """
        Callback chamado quando nova mensagem LIDAR chega

        Args:
            msg: LaserScan message com dados do LIDAR
        """
        # Pega distâncias do LIDAR
        ranges = msg.ranges

        # Encontra menor distância (obstáculo mais próximo)
        min_dist = min([r for r in ranges if r > 0])  # Ignora 0s (inválidos)

        if min_dist < 0.5:  # Menos de 50cm
            self.get_logger().warn(f'OBSTÁCULO PRÓXIMO! Distância: {min_dist:.2f}m')
        else:
            self.get_logger().info(f'Caminho livre. Mínimo: {min_dist:.2f}m')

def main(args=None):
    rclpy.init(args=args)
    node = SensorSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2.5 Exercício Prático 2: Controle Básico

Crie um node que integra publisher e subscriber para controle reativo:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class ControleReativo(Node):
    """
    Controle reativo: robô anda e desvia de obstáculos
    """

    def __init__(self):
        super().__init__('controle_reativo')

        # Publisher de velocidade
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber de LIDAR
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.callback_scan,
            10
        )

        # Timer para controle
        self.timer = self.create_timer(0.1, self.controlar)  # 10 Hz

        self.obstaculo_proximo = False
        self.distancia_minima = float('inf')

        self.get_logger().info('Controle reativo iniciado!')

    def callback_scan(self, msg):
        """Processa dados do LIDAR"""
        # Pega apenas leituras frontais (ex: 30 graus para cada lado)
        num_leituras = len(msg.ranges)
        angulo_frontal = 30  # graus

        # Calcula índices das leituras frontais
        indice_inicio = num_leituras // 2 - angulo_frontal
        indice_fim = num_leituras // 2 + angulo_frontal

        leituras_frente = msg.ranges[indice_inicio:indice_fim]

        # Filtra leituras válidas
        leituras_validas = [r for r in leituras_frente if r > 0]

        if leituras_validas:
            self.distancia_minima = min(leituras_validas)
            self.obstaculo_proximo = self.distancia_minima < 1.0  # 1 metro
        else:
            self.obstaculo_proximo = False

    def controlar(self):
        """Lógica de controle"""
        msg = Twist()

        if self.obstaculo_proximo:
            # Obstáculo detectado - para e gira
            msg.linear.x = 0.0
            msg.angular.z = 0.5  # Gira para esquerda
            self.get_logger().warn(f'Obstáculo a {self.distancia_minima:.2f}m - Girando!')
        else:
            # Caminho livre - anda para frente
            msg.linear.x = 0.3  # 0.3 m/s
            msg.angular.z = 0.0
            self.get_logger().info('Caminho livre - Andando...')

        self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ControleReativo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Parte 3: Integração ROS 2 + Webots (3-4h)

### 3.1 Instalação Webots com ROS 2

```bash
# Instalar Webots
sudo apt install wget
wget https://github.com/cyberbotics/webots/releases/download/R2023b/webots_2023b_amd64.deb
sudo apt install ./webots_2023b_amd64.deb

# Instalar pacotes ROS 2 para Webots
sudo apt install ros-humble-webots-ros2
```

### 3.2 Primeiro Robô no Webots com ROS 2

Crie arquivo `meu_mundo.wbt` (mundo Webots):

```
#VRML_SIM R2023b utf8

EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/backgrounds/protos/TexturedBackground.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/floors/protos/Floor.proto"

WorldInfo {
}
Viewpoint {
  position 0 -3 2
  orientation 0.5 0.5 0.7 1.8
}
TexturedBackground {
}
Floor {
  size 10 10
}
Robot {
  name "meu_robo"
  translation 0 0 0.5
  children [
    Shape {
      geometry Box {
        size 0.3 0.3 0.5
      }
    }
    HingeJoint {
      device [
        RotationalMotor {
          name "motor_esquerdo"
        }
      ]
    }
    HingeJoint {
      device [
        RotationalMotor {
          name "motor_direito"
        }
      ]
    }
  ]
  controller "<extern>"
}
```

Controller ROS 2:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from webots_ros2_driver.webots_controller import WebotsController

class WebotsRoboController(WebotsController):
    """Controlador Webots integrado com ROS 2"""

    def __init__(self):
        super().__init__('webots_controller')

        # Subscreve comandos de velocidade
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.callback_cmd_vel,
            10
        )

        # Pega referências dos motores Webots
        self.motor_esq = self.robot.getDevice('motor_esquerdo')
        self.motor_dir = self.robot.getDevice('motor_direito')

        # Configura motores para controle de velocidade
        self.motor_esq.setPosition(float('inf'))
        self.motor_dir.setPosition(float('inf'))
        self.motor_esq.setVelocity(0.0)
        self.motor_dir.setVelocity(0.0)

        self.get_logger().info('Webots controller iniciado!')

    def callback_cmd_vel(self, msg):
        """Converte Twist para velocidades de motor"""
        linear = msg.linear.x
        angular = msg.angular.z

        # Cinemática diferencial simplificada
        vel_esq = linear - angular
        vel_dir = linear + angular

        self.motor_esq.setVelocity(vel_esq)
        self.motor_dir.setVelocity(vel_dir)

        self.get_logger().info(f'Motores: E={vel_esq:.2f}, D={vel_dir:.2f}')

def main(args=None):
    rclpy.init(args=args)
    controller = WebotsRoboController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3.3 Executando

```bash
# Terminal 1: Inicia Webots com mundo
webots meu_mundo.wbt

# Terminal 2: Inicia controller ROS 2
source ~/ros2_ws/install/setup.bash
ros2 run meu_robo_py webots_controller

# Terminal 3: Publica comandos de velocidade
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

---

## Mini-Projeto: Robô que Segue Parede

Crie um node que faz o robô seguir uma parede usando LIDAR:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class SeguidorParede(Node):
    """
    Robô que segue parede usando LIDAR
    """

    def __init__(self):
        super().__init__('seguidor_parede')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.callback_scan, 10)

        self.distancia_desejada = 0.5  # 50cm da parede
        self.velocidade_linear = 0.3

        self.get_logger().info('Seguidor de parede iniciado!')

    def callback_scan(self, msg):
        """Processa LIDAR e calcula controle"""
        ranges = np.array(msg.ranges)

        # Divide LIDAR em regiões
        num_leituras = len(ranges)

        # Lado direito (assumimos parede à direita)
        lado_dir = ranges[0:num_leituras//4]

        # Frente
        frente = ranges[num_leituras//2 - 10:num_leituras//2 + 10]

        # Calcula distâncias médias
        dist_direita = np.mean([r for r in lado_dir if 0 < r < 10])
        dist_frente = np.min([r for r in frente if 0 < r < 10])

        # Lógica de controle
        msg_vel = Twist()

        # Controle proporcional para seguir parede
        erro = dist_direita - self.distancia_desejada
        kp = 2.0  # Ganho proporcional

        if dist_frente < 0.7:
            # Obstáculo à frente - gira à esquerda
            msg_vel.linear.x = 0.0
            msg_vel.angular.z = 0.5
            self.get_logger().warn('Obstáculo à frente!')
        else:
            # Segue parede
            msg_vel.linear.x = self.velocidade_linear
            msg_vel.angular.z = -kp * erro  # Negativo para corrigir
            self.get_logger().info(f'Distância direita: {dist_direita:.2f}m, Erro: {erro:.2f}')

        self.cmd_vel_pub.publish(msg_vel)

def main(args=None):
    rclpy.init(args=args)
    node = SeguidorParede()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Recursos Adicionais

### Documentação
- [ROS 2 Humble Docs](https://docs.ros.org/en/humble/)
- [Webots ROS 2 Interface](https://github.com/cyberbotics/webots_ros2)
- [Python Official Docs](https://docs.python.org/3/)

### Tutoriais
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [Real Python - Robotics](https://realpython.com/)

### Comunidade
- [ROS Discourse](https://discourse.ros.org/)
- [Webots Discord](https://discord.gg/cyberbotics)

---

## Troubleshooting - Erros Comuns

### Erro: "colcon: command not found"

```bash
# Solução
sudo apt install python3-colcon-common-extensions
```

### Erro: "Package not found" ao fazer colcon build

```bash
# Solução: limpar workspace e rebuildar
cd ~/ros2_ws
rm -rf build install log
colcon build
```

### Erro: Topic não aparece com `ros2 topic list`

```bash
# Verificar se node está rodando
ros2 node list

# Verificar se domínio ROS_DOMAIN_ID é o mesmo
echo $ROS_DOMAIN_ID
```

### Erro: "LIDAR não retorna dados"

Verifique se o sensor está habilitado no mundo Webots e se o topic name está correto.

---

## Checklist de Conclusão

- [ ] Ambiente Python configurado com virtualenv
- [ ] ROS 2 Humble instalado e funcionando
- [ ] Primeiro node ROS 2 criado e executado
- [ ] Publisher de velocidade implementado
- [ ] Subscriber de sensor implementado
- [ ] Integração Webots + ROS 2 funcionando
- [ ] Mini-projeto "Seguidor de Parede" concluído
- [ ] Código versionado no GitHub

---

## Próximo Módulo

No **Módulo 2.2: Sensores e Controle**, você vai:
- Trabalhar com diferentes tipos de sensores
- Processar imagens com OpenCV
- Implementar controle PID
- Criar comportamentos mais complexos

[→ Ir para Módulo 2.2]({{ '/niveis/nivel-2/modulo-2' | relative_url }}){: .btn .btn-primary}

---

**Dúvidas?** Participe da [comunidade FTH no Discord](#) ou abra uma issue no [GitHub](#).

**Última atualização:** 2025-10-29
