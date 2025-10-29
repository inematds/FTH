---
layout: page
title: "Módulo 2.3: Introdução a Aprendizado por Reforço"
permalink: /niveis/nivel-2/modulo-3/
---

# Módulo 2.3: Introdução a Aprendizado por Reforço

**Duração estimada:** 13-18 horas
**Nível:** Intermediário
**Pré-requisito:** Módulos 2.1 e 2.2 concluídos

---

## Objetivos de Aprendizado

Ao final deste módulo, você será capaz de:

- [ ] Compreender conceitos fundamentais de Aprendizado por Reforço
- [ ] Diferenciar RL de outros tipos de aprendizado de máquina
- [ ] Instalar e configurar Stable-Baselines3
- [ ] Criar ambientes Gym personalizados
- [ ] Treinar robôs usando algoritmo PPO
- [ ] Avaliar e visualizar curvas de aprendizado
- [ ] Testar modelos treinados em simulação
- [ ] Entender limitações e desafios do RL

---

## Parte 1: O que é Aprendizado por Reforço? (3-4h)

### 1.1 Aprendizado por Tentativa e Erro

Imagine aprender a andar de bicicleta:

- Você **tenta** pedalar (ação)
- Você **cai** ou **fica equilibrado** (feedback do ambiente)
- Você **ajusta** seu comportamento baseado no resultado
- Você **repete** até aprender

Isso é **Aprendizado por Reforço (RL)**!

```
           ┌─────────────────────────────┐
           │                             │
           │      APRENDIZADO POR        │
           │         REFORÇO             │
           │                             │
           │  ┌─────────┐   ┌─────────┐ │
           │  │ Agente  │   │Ambiente │ │
           │  │ (Robô)  │◄──┤ (Mundo) │ │
           │  └────┬────┘   └────▲────┘ │
           │       │             │       │
           │       │   Estado    │       │
           │       └─────────────┘       │
           │       │                     │
           │       │   Ação              │
           │       └──────►              │
           │                             │
           │       ◄──────               │
           │         Recompensa          │
           └─────────────────────────────┘
```

### 1.2 Componentes do RL

#### 1. Agente
O "aprendiz" que toma decisões (o robô humanoide).

#### 2. Ambiente
O mundo onde o agente atua (simulador, mundo real).

#### 3. Estado (State)
Informação atual do ambiente. Exemplos:
- Posição do robô: `[x=0.5, y=1.2, z=0.0]`
- Velocidade: `[vx=0.3, vy=0.0]`
- Orientação: `[roll=0, pitch=5°, yaw=90°]`
- Leituras de sensores: `[lidar=[0.8, 1.2, ...], camera=[pixels]]`

#### 4. Ação (Action)
Decisão que o agente toma. Exemplos:
- **Discreta:** "Andar frente", "Girar esquerda", "Parar"
- **Contínua:** Velocidade linear = 0.5 m/s, Angular = 0.2 rad/s

#### 5. Recompensa (Reward)
Feedback numérico que diz se ação foi boa ou ruim.

Exemplos:
```python
# Bom: robô andou para frente
recompensa = +1.0

# Ruim: robô bateu em obstáculo
recompensa = -10.0

# Neutro: robô parado
recompensa = 0.0
```

#### 6. Política (Policy)
Estratégia do agente: mapeamento de estados para ações.

```python
# Política simples (regra)
if distancia_obstaculo < 0.5:
    acao = "girar"
else:
    acao = "andar_frente"

# Política RL (rede neural)
acao = rede_neural.prever(estado)
```

### 1.3 RL vs Outras Técnicas

| Característica | RL | Aprendizado Supervisionado | Controle Tradicional |
|----------------|----|-----------------------------|----------------------|
| **Dados** | Aprende por tentativa | Precisa de dataset rotulado | Precisa de modelo matemático |
| **Feedback** | Recompensa (tarde) | Labels (imediato) | Erro de controle |
| **Exploração** | Explora novas ações | Não explora | Não explora |
| **Adaptação** | Adapta-se a mudanças | Fixo após treino | Fixo (tunning manual) |
| **Exemplo** | Robô aprende a andar | Classificar imagens | PID controller |

### 1.4 Algoritmos de RL

Existem dezenas de algoritmos. Os principais para robótica:

#### Q-Learning (clássico)
- Aprende valor de cada ação em cada estado
- Funciona bem em ambientes simples
- Difícil escalar para robôs complexos

#### Deep Q-Network (DQN)
- Q-Learning com redes neurais
- Usado em jogos Atari
- Melhor para ações discretas

#### Proximal Policy Optimization (PPO)
- **MUITO USADO EM ROBÓTICA**
- Aprende política diretamente
- Funciona com ações contínuas
- Estável e eficiente

#### Soft Actor-Critic (SAC)
- State-of-the-art para controle contínuo
- Mais complexo que PPO
- Muito eficiente em amostragem

**Neste módulo usaremos PPO!**

### 1.5 Exemplo Conceitual

Treinar robô para alcançar objetivo:

```
EPISÓDIO 1 (robô novato)
─────────────────────────
t=0:  Estado: [x=0, y=0, meta=[x=5, y=0]]
      Ação: andar_frente (velocidade aleatória)
      Recompensa: +0.1 (moveu na direção certa)

t=1:  Estado: [x=0.2, y=0]
      Ação: girar_esquerda (exploração)
      Recompensa: -0.1 (moveu na direção errada)

t=2:  Estado: [x=0.2, y=0.1]
      Ação: andar_frente
      Recompensa: +0.1

...

t=50: Estado: [x=2.3, y=1.5]
      Ação: parar (não chegou)
      Recompensa: -5.0 (não alcançou meta)

RECOMPENSA TOTAL: -10.3
─────────────────────────

EPISÓDIO 100 (robô aprendeu!)
─────────────────────────
t=0:  Estado: [x=0, y=0, meta=[x=5, y=0]]
      Ação: andar_frente (rede neural decidiu)
      Recompensa: +0.5

t=1:  Estado: [x=0.8, y=0]
      Ação: andar_frente
      Recompensa: +0.5

...

t=10: Estado: [x=5.0, y=0.0]
      Ação: parar (chegou!)
      Recompensa: +100.0 (ALCANÇOU META!)

RECOMPENSA TOTAL: +150.0
─────────────────────────
```

---

## Parte 2: Instalação e Setup (1-2h)

### 2.1 Instalando Stable-Baselines3

Stable-Baselines3 (SB3) é a biblioteca mais popular de RL para Python.

```bash
# Ative seu ambiente virtual
source ~/robotica/robo_env/bin/activate  # Linux/Mac
# ou
robo_env\Scripts\activate  # Windows

# Instale SB3 e dependências
pip install stable-baselines3[extra]
pip install gymnasium
pip install tensorboard

# Para visualização
pip install matplotlib
```

### 2.2 Primeiro Código RL - Cartpole

CartPole: balançar uma vara em um carrinho (ambiente clássico).

```python
import gymnasium as gym
from stable_baselines3 import PPO

# Cria ambiente
env = gym.make("CartPole-v1", render_mode="human")

# Cria agente PPO
model = PPO("MlpPolicy", env, verbose=1)

# Treina por 10.000 timesteps
print("Treinando...")
model.fit(total_timesteps=10000)

# Salva modelo
model.save("ppo_cartpole")
print("Modelo salvo!")

# Testa modelo treinado
print("Testando modelo...")
obs, info = env.reset()
for i in range(1000):
    action, _states = model.predict(obs, deterministic=True)
    obs, reward, terminated, truncated, info = env.step(action)
    env.render()

    if terminated or truncated:
        obs, info = env.reset()

env.close()
```

**Execute:**
```bash
python primeiro_rl.py
```

Você verá o CartPole aprendendo a se equilibrar!

### 2.3 Entendendo o Código

```python
# 1. Criar ambiente
env = gym.make("CartPole-v1")

# 2. Criar agente
model = PPO(
    "MlpPolicy",  # Política: Multi-Layer Perceptron (rede neural)
    env,          # Ambiente
    verbose=1     # Mostra progresso
)

# 3. Treinar
model.fit(total_timesteps=10000)

# 4. Prever ação
action, _states = model.predict(obs, deterministic=True)

# 5. Executar ação no ambiente
obs, reward, terminated, truncated, info = env.step(action)
```

### 2.4 Visualizando Aprendizado com TensorBoard

```python
from stable_baselines3 import PPO
import gymnasium as gym

# Cria ambiente
env = gym.make("CartPole-v1")

# Cria modelo com logging
model = PPO("MlpPolicy", env, verbose=1, tensorboard_log="./tensorboard_logs/")

# Treina
model.fit(total_timesteps=50000)
```

**Visualize no navegador:**
```bash
tensorboard --logdir ./tensorboard_logs/
# Abra http://localhost:6006
```

Você verá gráficos de:
- Recompensa ao longo do tempo
- Loss da rede neural
- Valor estimado

---

## Parte 3: Criando Ambiente Gym Personalizado (4-5h)

### 3.1 Estrutura de um Ambiente Gym

Todo ambiente Gym precisa de:

```python
import gymnasium as gym
from gymnasium import spaces
import numpy as np

class MeuAmbiente(gym.Env):
    """Ambiente personalizado seguindo interface gym.Env"""

    def __init__(self):
        super(MeuAmbiente, self).__init__()

        # Define espaço de ações
        # Exemplo: 2 ações contínuas (velocidade linear e angular)
        self.action_space = spaces.Box(
            low=np.array([-1.0, -1.0]),
            high=np.array([1.0, 1.0]),
            dtype=np.float32
        )

        # Define espaço de observações
        # Exemplo: posição x, y, ângulo
        self.observation_space = spaces.Box(
            low=np.array([-10, -10, -np.pi]),
            high=np.array([10, 10, np.pi]),
            dtype=np.float32
        )

    def reset(self, seed=None, options=None):
        """
        Reseta ambiente para estado inicial

        Returns:
            observation: Estado inicial
            info: Dict com informações extras
        """
        super().reset(seed=seed)

        # Inicializa estado
        self.state = np.array([0.0, 0.0, 0.0])  # [x, y, theta]

        return self.state, {}

    def step(self, action):
        """
        Executa ação e retorna resultado

        Args:
            action: Ação a ser executada

        Returns:
            observation: Novo estado
            reward: Recompensa
            terminated: Se episódio terminou
            truncated: Se episódio foi truncado (timeout)
            info: Dict com informações extras
        """
        # Executa ação (simula movimento)
        vel_linear = action[0]
        vel_angular = action[1]

        # Atualiza estado (simplificado)
        dt = 0.1
        self.state[0] += vel_linear * np.cos(self.state[2]) * dt  # x
        self.state[1] += vel_linear * np.sin(self.state[2]) * dt  # y
        self.state[2] += vel_angular * dt  # theta

        # Calcula recompensa
        reward = self._calcular_recompensa()

        # Verifica se terminou
        terminated = self._check_terminado()
        truncated = False

        return self.state, reward, terminated, truncated, {}

    def _calcular_recompensa(self):
        """Implementa lógica de recompensa"""
        # Exemplo simples
        return -0.1  # Penalidade por timestep

    def _check_terminado(self):
        """Verifica se episódio terminou"""
        # Exemplo: termina se sair de limites
        x, y, _ = self.state
        if abs(x) > 10 or abs(y) > 10:
            return True
        return False

    def render(self, mode='human'):
        """Renderiza ambiente (opcional)"""
        print(f"Estado: x={self.state[0]:.2f}, y={self.state[1]:.2f}, theta={self.state[2]:.2f}")
```

### 3.2 Ambiente: Robô Alcança Objetivo

Vamos criar um ambiente onde robô precisa alcançar um ponto no espaço 2D:

```python
import gymnasium as gym
from gymnasium import spaces
import numpy as np
import matplotlib.pyplot as plt

class RoboAlcancaObjetivo(gym.Env):
    """
    Ambiente: Robô 2D precisa alcançar objetivo

    Observação:
        - Posição robô [x, y]
        - Posição objetivo [goal_x, goal_y]
        - Distância até objetivo

    Ação:
        - Velocidade linear [-1, 1]
        - Velocidade angular [-1, 1]

    Recompensa:
        - +100 se alcançar objetivo
        - -distancia_até_objetivo a cada step
        - -0.1 por timestep (incentiva eficiência)
    """

    def __init__(self, render_mode=None):
        super(RoboAlcancaObjetivo, self).__init__()

        # Ações: [vel_linear, vel_angular]
        self.action_space = spaces.Box(
            low=np.array([-1.0, -1.0]),
            high=np.array([1.0, 1.0]),
            dtype=np.float32
        )

        # Observações: [x, y, theta, goal_x, goal_y]
        self.observation_space = spaces.Box(
            low=np.array([-10, -10, -np.pi, -10, -10]),
            high=np.array([10, 10, np.pi, 10, 10]),
            dtype=np.float32
        )

        self.render_mode = render_mode

        # Parâmetros
        self.max_steps = 200
        self.goal_threshold = 0.3  # Distância para considerar "alcançado"

        # Estado
        self.pos = np.array([0.0, 0.0])  # [x, y]
        self.theta = 0.0
        self.goal = np.array([5.0, 5.0])
        self.steps = 0

        # Para render
        if render_mode == "human":
            self.fig, self.ax = plt.subplots()
            plt.ion()

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        # Posição inicial aleatória
        self.pos = np.random.uniform(-2, 2, size=2)
        self.theta = np.random.uniform(-np.pi, np.pi)

        # Objetivo aleatório
        self.goal = np.random.uniform(3, 7, size=2)

        self.steps = 0

        return self._get_obs(), {}

    def _get_obs(self):
        """Retorna observação atual"""
        return np.array([
            self.pos[0],
            self.pos[1],
            self.theta,
            self.goal[0],
            self.goal[1]
        ], dtype=np.float32)

    def step(self, action):
        # Extrai ações
        vel_linear = action[0] * 0.5  # Max 0.5 m/s
        vel_angular = action[1] * 1.0  # Max 1.0 rad/s

        # Atualiza posição (cinemática diferencial)
        dt = 0.1
        self.pos[0] += vel_linear * np.cos(self.theta) * dt
        self.pos[1] += vel_linear * np.sin(self.theta) * dt
        self.theta += vel_angular * dt

        # Normaliza theta para [-pi, pi]
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))

        self.steps += 1

        # Calcula distância até objetivo
        distancia = np.linalg.norm(self.pos - self.goal)

        # Calcula recompensa
        reward = -distancia  # Quanto mais longe, pior

        # Bônus se alcançou
        terminated = False
        if distancia < self.goal_threshold:
            reward += 100.0
            terminated = True

        # Penalidade se saiu do mapa
        if np.any(np.abs(self.pos) > 10):
            reward -= 50.0
            terminated = True

        # Trunca se passou do limite de steps
        truncated = self.steps >= self.max_steps

        # Pequena penalidade por timestep (incentiva velocidade)
        reward -= 0.1

        if self.render_mode == "human":
            self.render()

        return self._get_obs(), reward, terminated, truncated, {}

    def render(self):
        """Renderiza ambiente"""
        if self.render_mode == "human":
            self.ax.clear()

            # Desenha robô
            self.ax.plot(self.pos[0], self.pos[1], 'bo', markersize=10, label='Robô')

            # Desenha direção
            dx = 0.3 * np.cos(self.theta)
            dy = 0.3 * np.sin(self.theta)
            self.ax.arrow(self.pos[0], self.pos[1], dx, dy, head_width=0.2, color='b')

            # Desenha objetivo
            self.ax.plot(self.goal[0], self.goal[1], 'r*', markersize=20, label='Objetivo')

            # Desenha círculo de alcance
            circle = plt.Circle(self.goal, self.goal_threshold, color='r', fill=False, linestyle='--')
            self.ax.add_patch(circle)

            # Configurações
            self.ax.set_xlim(-10, 10)
            self.ax.set_ylim(-10, 10)
            self.ax.set_aspect('equal')
            self.ax.grid(True)
            self.ax.legend()
            self.ax.set_title(f'Step: {self.steps}')

            plt.pause(0.01)

    def close(self):
        if self.render_mode == "human":
            plt.close()
```

### 3.3 Treinando o Ambiente Personalizado

```python
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env

# Verifica se ambiente está correto
env = RoboAlcancaObjetivo()
check_env(env)
print("Ambiente válido!")

# Cria modelo
model = PPO(
    "MlpPolicy",
    env,
    verbose=1,
    tensorboard_log="./logs_robo_objetivo/",
    learning_rate=0.0003,
    n_steps=2048,
    batch_size=64,
    n_epochs=10,
    gamma=0.99,
    gae_lambda=0.95
)

# Treina
print("Iniciando treinamento...")
model.fit(total_timesteps=100000)

# Salva
model.save("ppo_robo_alcanca_objetivo")
print("Modelo treinado e salvo!")

# Testa
print("\nTestando modelo...")
env_teste = RoboAlcancaObjetivo(render_mode="human")
obs, info = env_teste.reset()

for _ in range(500):
    action, _states = model.predict(obs, deterministic=True)
    obs, reward, terminated, truncated, info = env_teste.step(action)

    if terminated or truncated:
        print("Episódio terminou!")
        obs, info = env_teste.reset()

env_teste.close()
```

---

## Parte 4: Entendendo e Visualizando Resultados (2-3h)

### 4.1 Curvas de Aprendizado

Durante o treinamento, monitore:

```
┌─────────────────────────────────────┐
│  RECOMPENSA MÉDIA vs TIMESTEPS      │
│                                     │
│  100│            ╱────────          │
│     │         ╱                     │
│   50│      ╱                        │
│     │   ╱                           │
│    0├──────────────────────         │
│     0    25k   50k   75k  100k     │
│           Timesteps                 │
└─────────────────────────────────────┘

Interpretação:
- Subindo: Aprendendo ✅
- Platô: Convergiu ✅
- Descendo: Problema ❌
- Muito ruidoso: Aumentar batch_size
```

### 4.2 Avaliando Modelo

```python
from stable_baselines3.common.evaluation import evaluate_policy

# Carrega modelo
model = PPO.load("ppo_robo_alcanca_objetivo")

# Cria ambiente
env = RoboAlcancaObjetivo()

# Avalia por 10 episódios
mean_reward, std_reward = evaluate_policy(
    model,
    env,
    n_eval_episodes=10,
    deterministic=True
)

print(f"Recompensa média: {mean_reward:.2f} +/- {std_reward:.2f}")

# Análise
if mean_reward > 80:
    print("Modelo EXCELENTE! ✅")
elif mean_reward > 50:
    print("Modelo BOM! Pode melhorar.")
elif mean_reward > 0:
    print("Modelo MEDIANO. Precisa mais treino.")
else:
    print("Modelo RUIM. Revise recompensas e hiperparâmetros.")
```

### 4.3 Visualizando Trajetórias

```python
import matplotlib.pyplot as plt

def visualizar_trajetorias(model, env, num_episodios=5):
    """Plota trajetórias de múltiplos episódios"""

    plt.figure(figsize=(10, 10))

    for ep in range(num_episodios):
        obs, info = env.reset()
        trajetoria = [obs[:2]]  # Guarda [x, y]

        for _ in range(200):
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            trajetoria.append(obs[:2])

            if terminated or truncated:
                break

        trajetoria = np.array(trajetoria)

        # Plota trajetória
        plt.plot(trajetoria[:, 0], trajetoria[:, 1], '-o', alpha=0.6, label=f'Ep {ep+1}')

        # Plota início e fim
        plt.plot(trajetoria[0, 0], trajetoria[0, 1], 'go', markersize=10)
        plt.plot(trajetoria[-1, 0], trajetoria[-1, 1], 'ro', markersize=10)

        # Plota objetivo
        goal = obs[3:5]
        plt.plot(goal[0], goal[1], 'r*', markersize=20)

    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('Trajetórias do Robô')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()

# Uso
model = PPO.load("ppo_robo_alcanca_objetivo")
env = RoboAlcancaObjetivo()
visualizar_trajetorias(model, env, num_episodios=5)
```

### 4.4 Comparando Antes e Depois do Treino

```python
def comparar_desempenho():
    """Compara robô aleatório vs treinado"""

    env = RoboAlcancaObjetivo()
    model = PPO.load("ppo_robo_alcanca_objetivo")

    # Testa robô aleatório
    print("=== ROBÔ ALEATÓRIO ===")
    recompensas_aleatorio = []
    for _ in range(10):
        obs, info = env.reset()
        recompensa_total = 0
        for _ in range(200):
            action = env.action_space.sample()  # Ação aleatória
            obs, reward, terminated, truncated, info = env.step(action)
            recompensa_total += reward
            if terminated or truncated:
                break
        recompensas_aleatorio.append(recompensa_total)

    # Testa robô treinado
    print("=== ROBÔ TREINADO ===")
    recompensas_treinado = []
    for _ in range(10):
        obs, info = env.reset()
        recompensa_total = 0
        for _ in range(200):
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            recompensa_total += reward
            if terminated or truncated:
                break
        recompensas_treinado.append(recompensa_total)

    # Resultados
    print(f"\nAleatório: {np.mean(recompensas_aleatorio):.2f} +/- {np.std(recompensas_aleatorio):.2f}")
    print(f"Treinado:  {np.mean(recompensas_treinado):.2f} +/- {np.std(recompensas_treinado):.2f}")

    melhoria = np.mean(recompensas_treinado) - np.mean(recompensas_aleatorio)
    print(f"\nMelhoria: +{melhoria:.2f} ({melhoria/abs(np.mean(recompensas_aleatorio))*100:.1f}%)")

comparar_desempenho()
```

---

## Parte 5: Integração com ROS 2 e Simulador (3-4h)

### 5.1 Ambiente Gym com ROS 2

Integre ambiente Gym com robô real no ROS 2:

```python
import gymnasium as gym
from gymnasium import spaces
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class RoboROS2Env(gym.Env, Node):
    """
    Ambiente Gym que controla robô real via ROS 2
    """

    def __init__(self):
        # Inicializa Gym Env
        super(RoboROS2Env, self).__init__()

        # Inicializa ROS 2 Node
        rclpy.init()
        Node.__init__(self, 'robo_rl_env')

        # Espaços
        self.action_space = spaces.Box(
            low=np.array([-1.0, -1.0]),
            high=np.array([1.0, 1.0]),
            dtype=np.float32
        )

        # Observação: [dist_frente, dist_esq, dist_dir, vel_x, vel_angular]
        self.observation_space = spaces.Box(
            low=np.array([0, 0, 0, -2, -2]),
            high=np.array([10, 10, 10, 2, 2]),
            dtype=np.float32
        )

        # Publishers e Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.callback_scan, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.callback_odom, 10)

        # Estado
        self.scan_data = None
        self.odom_data = None

        self.get_logger().info('Ambiente RL-ROS2 iniciado!')

    def callback_scan(self, msg):
        """Recebe dados LIDAR"""
        ranges = np.array(msg.ranges)
        ranges = np.clip(ranges, msg.range_min, msg.range_max)

        # Extrai 3 regiões: frente, esquerda, direita
        num_readings = len(ranges)
        self.dist_frente = np.min(ranges[num_readings//2-10:num_readings//2+10])
        self.dist_esq = np.min(ranges[3*num_readings//4:num_readings])
        self.dist_dir = np.min(ranges[0:num_readings//4])

    def callback_odom(self, msg):
        """Recebe odometria"""
        self.vel_x = msg.twist.twist.linear.x
        self.vel_angular = msg.twist.twist.angular.z

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        # Para robô
        msg = Twist()
        self.cmd_vel_pub.publish(msg)

        # Aguarda dados dos sensores
        while self.scan_data is None or self.odom_data is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        return self._get_obs(), {}

    def _get_obs(self):
        """Retorna observação atual"""
        return np.array([
            self.dist_frente,
            self.dist_esq,
            self.dist_dir,
            self.vel_x,
            self.vel_angular
        ], dtype=np.float32)

    def step(self, action):
        # Publica ação
        msg = Twist()
        msg.linear.x = float(action[0]) * 0.5  # Max 0.5 m/s
        msg.angular.z = float(action[1]) * 1.0  # Max 1.0 rad/s
        self.cmd_vel_pub.publish(msg)

        # Aguarda próximo estado
        rclpy.spin_once(self, timeout_sec=0.1)

        # Calcula recompensa
        reward = 0.0

        # Recompensa por andar para frente
        if self.dist_frente > 1.0:
            reward += self.vel_x  # Quanto mais rápido, melhor

        # Penalidade por colidir
        if self.dist_frente < 0.3:
            reward -= 10.0
            terminated = True
        else:
            terminated = False

        truncated = False

        return self._get_obs(), reward, terminated, truncated, {}

    def close(self):
        # Para robô
        msg = Twist()
        self.cmd_vel_pub.publish(msg)
        rclpy.shutdown()
```

### 5.2 Treinando com Robô Real

```python
from stable_baselines3 import PPO

# Cria ambiente ROS 2
env = RoboROS2Env()

# Treina
model = PPO("MlpPolicy", env, verbose=1)
model.fit(total_timesteps=50000)

# Salva
model.save("ppo_robo_ros2")
```

**ATENÇÃO:** Treinar com robô real é **LENTO** e pode **danificar hardware**. Sempre:
1. Treine primeiro em simulação
2. Teste em ambiente seguro
3. Tenha botão de emergência

---

## Parte 6: Hiperparâmetros e Troubleshooting (1-2h)

### 6.1 Principais Hiperparâmetros PPO

```python
model = PPO(
    "MlpPolicy",
    env,

    # Taxa de aprendizado
    learning_rate=3e-4,  # Padrão. Reduza se instável (1e-4), aumente se lento (1e-3)

    # Quantos steps coletar antes de atualizar
    n_steps=2048,  # Maior = mais estável, mas mais lento

    # Tamanho do batch para treino
    batch_size=64,  # Múltiplo de n_steps. Maior = mais estável

    # Quantas épocas treinar com dados coletados
    n_epochs=10,  # Padrão. Mais épocas = mais treino por dados

    # Discount factor (quanto valoriza recompensas futuras)
    gamma=0.99,  # 0.9-0.999. Maior = valoriza mais o futuro

    # Generalized Advantage Estimation
    gae_lambda=0.95,  # 0.9-0.99. Controla viés vs variância

    # Clip range para atualização de política
    clip_range=0.2,  # Padrão. Previne mudanças drásticas

    verbose=1
)
```

### 6.2 Problemas Comuns

#### Problema 1: Recompensa não sobe

**Causas:**
- Recompensas mal projetadas
- Learning rate muito alto
- Ações não têm efeito real no ambiente

**Soluções:**
```python
# 1. Verifique recompensas
print(f"Recompensa mínima possível: {min_reward}")
print(f"Recompensa máxima possível: {max_reward}")
print(f"Recompensa de ações aleatórias: {random_reward}")

# 2. Reduza learning rate
model = PPO("MlpPolicy", env, learning_rate=1e-4)

# 3. Simplifique ambiente
# Torne mais fácil obter recompensas positivas
```

#### Problema 2: Aprendizado muito lento

**Soluções:**
```python
# 1. Aumente paralelização
from stable_baselines3.common.vec_env import SubprocVecEnv

# Cria 4 ambientes em paralelo
env = SubprocVecEnv([lambda: RoboAlcancaObjetivo() for _ in range(4)])

model = PPO("MlpPolicy", env, verbose=1)
model.fit(total_timesteps=100000)  # 4x mais rápido!

# 2. Aumente learning rate
model = PPO("MlpPolicy", env, learning_rate=1e-3)

# 3. Aumente n_steps
model = PPO("MlpPolicy", env, n_steps=4096)
```

#### Problema 3: Modelo instável (performance cai)

**Soluções:**
```python
# 1. Reduza learning rate
model = PPO("MlpPolicy", env, learning_rate=1e-5)

# 2. Aumente clip_range
model = PPO("MlpPolicy", env, clip_range=0.1)

# 3. Salve checkpoints frequentemente
from stable_baselines3.common.callbacks import CheckpointCallback

checkpoint_callback = CheckpointCallback(
    save_freq=10000,
    save_path='./checkpoints/',
    name_prefix='ppo_model'
)

model.fit(total_timesteps=100000, callback=checkpoint_callback)
```

### 6.3 Design de Recompensas

**Princípios:**

1. **Recompensa deve refletir objetivo**
   ```python
   # BOM: Recompensa pela distância ao objetivo
   reward = -distancia_objetivo

   # RUIM: Recompensa fixa por timestep
   reward = 1.0
   ```

2. **Use recompensas densas, não esparsas**
   ```python
   # RUIM (esparsa): Só recompensa no final
   reward = 100 if alcançou_objetivo else 0

   # BOM (densa): Recompensa contínua
   reward = -distancia_objetivo  # Quanto mais perto, melhor
   if alcançou_objetivo:
       reward += 100
   ```

3. **Normalize recompensas**
   ```python
   # Mantenha recompensas em [-1, 1] ou [-100, 100]
   reward = np.clip(reward, -100, 100)
   ```

4. **Penalize comportamentos ruins**
   ```python
   # Colidiu
   reward -= 50

   # Ficou parado
   if velocidade < 0.1:
       reward -= 1
   ```

---

## Mini-Projeto Final: Robô Desvio de Obstáculos com RL

**Objetivo:** Criar ambiente e treinar robô para:
1. Navegar em ambiente com obstáculos
2. Alcançar objetivo o mais rápido possível
3. Não colidir

**Checklist:**
- [ ] Criar ambiente Gym com obstáculos
- [ ] Definir observações (LIDAR + posição + objetivo)
- [ ] Definir ações (velocidades)
- [ ] Projetar função de recompensa
- [ ] Treinar com PPO por 200k timesteps
- [ ] Avaliar desempenho
- [ ] Visualizar trajetórias
- [ ] Comparar com controle reativo (Módulo 2.2)
- [ ] Documentar resultados

---

## Recursos Adicionais

### Documentação
- [Stable-Baselines3 Docs](https://stable-baselines3.readthedocs.io/)
- [Gymnasium Docs](https://gymnasium.farama.org/)
- [Spinning Up in Deep RL (OpenAI)](https://spinningup.openai.com/)

### Cursos
- [Deep RL Course - Hugging Face](https://huggingface.co/learn/deep-rl-course/)
- [RL Specialization - Coursera](https://www.coursera.org/specializations/reinforcement-learning)

### Papers
- [PPO Paper (Schulman et al., 2017)](https://arxiv.org/abs/1707.06347)
- [Sim-to-Real Transfer](https://arxiv.org/abs/1803.07067)

---

## Troubleshooting

### Erro: "Box bound precision lowered by casting to float32"

Ignore - é só um aviso. Ou use `dtype=np.float32` explicitamente.

### Erro: "AssertionError: The action space must be Box"

Você está tentando usar espaço de ações discreto com algoritmo que requer contínuo. Use DQN para discreto ou Box action_space para PPO.

### Treinamento não converge

1. Simplifique ambiente (menos observações, menos ações)
2. Verifique se recompensas fazem sentido
3. Teste com CartPole primeiro para validar código
4. Aumente total_timesteps

---

## Checklist de Conclusão

- [ ] Compreender conceitos de RL (agente, ambiente, recompensa)
- [ ] Instalar Stable-Baselines3
- [ ] Treinar CartPole
- [ ] Criar ambiente Gym personalizado
- [ ] Treinar "Robô Alcança Objetivo"
- [ ] Avaliar modelo com evaluate_policy
- [ ] Visualizar curvas de aprendizado (TensorBoard)
- [ ] Visualizar trajetórias
- [ ] Integrar Gym com ROS 2 (conceitual)
- [ ] Fazer tunning de hiperparâmetros
- [ ] Mini-projeto "Desvio de Obstáculos" concluído

---

## Próximo Nível

Parabéns! Você completou o **Nível 2: Criador**.

Você agora sabe:
- Programar em Python e ROS 2
- Trabalhar com sensores e controle
- Implementar Aprendizado por Reforço

No **Nível 3: Desenvolvedor**, você vai:
- Algoritmos avançados (SAC, TD3)
- Visão computacional profunda
- Large Behavior Models (LBMs)
- Sim-to-Real transfer

[→ Ir para Nível 3](/niveis/nivel-3/)

---

## Certificado Nível 2

Ao concluir todos os módulos e o projeto final, você receberá:

**Certificado Digital: "Criador em Robótica Humanoide"**
- Validado pela comunidade FTH
- Compartilhável no LinkedIn
- Acesso ao Nível 3 desbloqueado

---

**Dúvidas?** Participe da [comunidade FTH no Discord](#) ou abra uma issue no [GitHub](#).

**Última atualização:** 2025-10-29
