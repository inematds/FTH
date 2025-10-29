---
layout: page
title: "Módulo 4.2: Sim-to-Real Transfer"
permalink: /niveis/nivel-4/modulo-2/
---

# 🤖 Módulo 4.2: Sim-to-Real Transfer

**Da simulação perfeita para o mundo caótico: domine a arte de transferir IA para robôs reais**

---

## 📋 Informações do Módulo

| Atributo | Detalhes |
|----------|----------|
| **Duração estimada** | 10-15 horas |
| **Nível** | Profissional |
| **Pré-requisitos** | Nível 3 + Módulo 4.1 |
| **Tipo** | Técnico + Prático |

---

## 🎯 Objetivos de Aprendizado

Ao completar este módulo, você será capaz de:

- ✅ Entender o "domain gap" entre simulação e realidade
- ✅ Aplicar técnicas de domain randomization
- ✅ Realizar system identification de robôs reais
- ✅ Fine-tunar políticas treinadas em simulação
- ✅ Fazer deploy seguro de IA em hardware
- ✅ Debugar e troubleshoot problemas de Sim2Real
- ✅ Monitorar robôs em produção

---

## 📚 Conteúdo Teórico

### 1. O Desafio do Sim-to-Real

#### 1.1 Por que Políticas Falham no Mundo Real?

Você treinou seu robô perfeitamente no Isaac Sim. Ele caminha, desvia de obstáculos, pega objetos. Taxa de sucesso: 95%.

**Aí você transfere para o robô real e...**

❌ Ele cai no primeiro passo
❌ Sensores retornam dados ruidosos
❌ Latência de rede atrasa comandos
❌ Bateria fraca muda dinâmica
❌ Piso escorregadio vs simulado

**Taxa de sucesso no real:** 20%

---

#### 1.2 O "Domain Gap" (Lacuna de Domínio)

**Diferenças entre Simulação e Realidade:**

| Aspecto | Simulação | Realidade |
|---------|-----------|-----------|
| **Física** | Deterministíca, perfeita | Ruidosa, imperfeita |
| **Sensores** | Zero ruído | Ruído, drift, falhas |
| **Atuadores** | Resposta instantânea | Latência, backlash |
| **Ambiente** | Controlado | Imprevisível |
| **Computação** | Ilimitada | Limitada (bateria, CPU) |
| **Tempo** | Acelerável (10x) | Tempo real |

**Exemplo concreto:**

```python
# Simulação
joint_position = robot.get_joint_position("left_knee")
# Retorna: 1.5708 (sempre exato)

# Realidade
joint_position = robot.get_joint_position("left_knee")
# Retorna: 1.5701, 1.5715, 1.5693, ... (varia)
# + Às vezes retorna None (sensor perdeu sinal)
# + Latência de 20-50ms
```

---

#### 1.3 Abordagens para Sim2Real

**Abordagem 1: Simulação Ultra-Realista**

Fazer a simulação ser tão real quanto possível.

**Prós:**
- ✅ Transfere melhor
- ✅ Menos ajustes no real

**Contras:**
- ❌ Muito custoso computacionalmente
- ❌ Impossível replicar TODA realidade
- ❌ Treino mais lento

---

**Abordagem 2: Domain Randomization**

Tornar a simulação propositalmente DIVERSA e ruidosa.

**Prós:**
- ✅ Política aprende a ser robusta
- ✅ Generaliza melhor
- ✅ Mais usado na prática

**Contras:**
- ❌ Requer mais amostras de treino
- ❌ Pode ser "conservadora" demais

---

**Abordagem 3: Híbrido (Sim + Fine-tuning no Real)**

Treina na simulação, ajusta com dados reais.

**Prós:**
- ✅ Melhor de dois mundos
- ✅ Menos dados reais necessários

**Contras:**
- ❌ Requer acesso ao robô real
- ❌ Risco de overfit no ambiente específico

---

### 2. Domain Randomization

#### 2.1 Conceito

**Ideia:** Se a política funciona em 1000 versões diferentes da simulação, ela funcionará na realidade (que é apenas "mais uma versão").

**Parâmetros para Randomizar:**

**A) Física**
```python
# A cada episódio, variar:
gravity = random.uniform(9.6, 10.0)  # Terra varia por altitude
friction = random.uniform(0.3, 1.2)  # Piso liso vs áspero
mass = random.uniform(0.9, 1.1) * nominal_mass  # Tolerância fabricação
```

**B) Visual**
```python
# Câmera / Visão
lighting = random.choice(['bright', 'dim', 'mixed'])
camera_noise = add_gaussian_noise(image, std=random.uniform(0, 0.1))
color_shift = random_hue_saturation(image)
occlusion = randomly_block_parts_of_image(image, prob=0.1)
```

**C) Sensores**
```python
# IMU (equilíbrio)
imu_reading = true_value + random.normal(0, 0.05)  # Ruído
imu_drift = accumulated_error * timestep  # Drift ao longo do tempo

# Encoders dos motores
encoder_delay = random.uniform(0, 50)  # ms de latência
encoder_dropout = (random.random() < 0.02)  # 2% de falha
```

**D) Atuadores**
```python
# Motores
motor_force = command * random.uniform(0.95, 1.05)  # Variação
motor_delay = random.uniform(10, 30)  # ms
backlash = random.uniform(0, 2)  # graus de folga
```

---

#### 2.2 Implementação no Isaac Sim

**Código de exemplo:**

```python
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
import numpy as np

class DomainRandomizer:
    def __init__(self, world: World):
        self.world = world

    def randomize_physics(self):
        """Randomiza parâmetros físicos do ambiente"""

        # 1. Gravidade (±5%)
        gravity_scale = np.random.uniform(0.95, 1.05)
        self.world.get_physics_context().set_gravity(
            -9.81 * gravity_scale
        )

        # 2. Fricção do chão (0.3 a 1.2)
        ground = self.world.scene.get_object("ground_plane")
        friction = np.random.uniform(0.3, 1.2)
        ground.get_applied_physics_material().set_static_friction(friction)
        ground.get_applied_physics_material().set_dynamic_friction(friction)

        # 3. Massa do robô (±10%)
        robot = self.world.scene.get_object("bumi")
        for link in robot.get_links():
            original_mass = link.get_mass()
            new_mass = original_mass * np.random.uniform(0.9, 1.1)
            link.set_mass(new_mass)

    def randomize_visuals(self):
        """Randomiza aparência visual"""

        # 1. Iluminação
        lights = self.world.stage.GetPrimAtPath("/World/Lights")
        intensity = np.random.uniform(500, 3000)
        for light in lights.GetChildren():
            light.GetAttribute("intensity").Set(intensity)

        # 2. Cor do chão
        ground = self.world.stage.GetPrimAtPath("/World/ground")
        color = np.random.uniform(0.2, 0.9, size=3)  # RGB
        ground.GetAttribute("color").Set(color)

        # 3. Posição de objetos aleatórios (distratores)
        for i in range(np.random.randint(0, 5)):
            pos = np.random.uniform(-2, 2, size=3)
            pos[2] = 0.5  # altura fixa
            self.spawn_random_object(pos)

    def randomize_sensors(self, sensor_data):
        """Adiciona ruído a leituras de sensores"""

        # 1. IMU (acelerômetro + giroscópio)
        imu_noise = np.random.normal(0, 0.05, size=sensor_data['imu'].shape)
        sensor_data['imu'] += imu_noise

        # 2. Encoders (posição das juntas)
        encoder_noise = np.random.normal(0, 0.01, size=sensor_data['joints'].shape)
        sensor_data['joints'] += encoder_noise

        # 3. Dropout (2% de chance de perder leitura)
        if np.random.random() < 0.02:
            sensor_data['joints'] = None

        # 4. Câmera
        if 'camera' in sensor_data:
            # Adiciona ruído gaussiano
            noise = np.random.normal(0, 10, size=sensor_data['camera'].shape)
            sensor_data['camera'] = np.clip(sensor_data['camera'] + noise, 0, 255)

        return sensor_data

    def randomize_actuators(self, action):
        """Simula imperfeições dos motores"""

        # 1. Força variável (±5%)
        force_variation = np.random.uniform(0.95, 1.05, size=action.shape)
        action = action * force_variation

        # 2. Latência (10-30ms)
        # (requer buffer de ações passadas)
        delay_steps = np.random.randint(1, 3)  # 1-3 steps @ 100Hz = 10-30ms
        action = self.action_buffer[-delay_steps]

        # 3. Saturation (motores têm limite)
        action = np.clip(action, -1.0, 1.0)

        # 4. Dead zone (pequenos comandos são ignorados)
        action[np.abs(action) < 0.05] = 0

        return action

# Uso durante treino
world = World()
randomizer = DomainRandomizer(world)

for episode in range(10000):
    # A cada episódio, randomizar tudo
    randomizer.randomize_physics()
    randomizer.randomize_visuals()

    world.reset()

    for step in range(1000):
        # Observação com ruído
        obs = get_observations()
        obs = randomizer.randomize_sensors(obs)

        # Ação da política
        action = policy.predict(obs)

        # Ação com imperfeições
        action = randomizer.randomize_actuators(action)

        # Executa
        world.step(action)
```

---

#### 2.3 Boas Práticas de Randomização

**1. Comece Simples, Aumente Gradualmente**

```python
# Fase 1: Sem randomização (baseline)
randomization_level = 0.0

# Fase 2: Randomização leve
randomization_level = 0.3

# Fase 3: Randomização agressiva
randomization_level = 1.0
```

**2. Randomize o que Importa**

Não randomize tudo cegamente. Foque nas diferenças reais.

**Exemplo:**
- ✅ Randomizar fricção (pisos variam muito)
- ✅ Randomizar latência (Wi-Fi varia)
- ❌ Randomizar cores aleatoriamente (se seu robô sempre vai atuar em escritórios cinzas)

**3. Valide com "Sanity Check"**

Grave vídeos da simulação randomizada. Parecem plausíveis?

- ✅ Robô caminhando em pisos diferentes: OK
- ❌ Robô flutuando porque gravidade ficou negativa: NÃO OK

---

### 3. System Identification

#### 3.1 O que é?

**System Identification:** Processo de estimar os parâmetros reais do robô físico.

**Por que fazer?**
- Simulação usa parâmetros nominais (do manual)
- Robô real tem variações de fabricação
- Desgaste altera parâmetros ao longo do tempo

**Parâmetros a Identificar:**

| Parâmetro | Nominal (Manual) | Real (Medido) |
|-----------|------------------|---------------|
| Massa total | 8.0 kg | 8.3 kg |
| Fricção juntas | 0.01 Nm | 0.015 Nm |
| Inércia link | 0.05 kg·m² | 0.053 kg·m² |
| Latência motores | 5 ms | 18 ms |
| Constante motor | 0.05 Nm/A | 0.048 Nm/A |

---

#### 3.2 Técnicas de Identificação

**Método 1: Análise de Resposta ao Degrau**

Aplique comando fixo, observe resposta.

```python
import numpy as np
import matplotlib.pyplot as plt

def identify_motor_response(robot, joint_name):
    """
    Identifica constante de tempo e ganho do motor
    """
    # 1. Aplicar comando step
    robot.set_joint_effort(joint_name, 1.0)  # 100% torque

    # 2. Registrar posição ao longo do tempo
    positions = []
    times = []

    for t in np.arange(0, 2.0, 0.01):  # 2 segundos @ 100Hz
        pos = robot.get_joint_position(joint_name)
        positions.append(pos)
        times.append(t)
        time.sleep(0.01)

    # 3. Fit modelo de 1ª ordem: y(t) = K(1 - e^(-t/τ))
    from scipy.optimize import curve_fit

    def model(t, K, tau):
        return K * (1 - np.exp(-t / tau))

    params, _ = curve_fit(model, times, positions)
    K, tau = params

    print(f"Ganho (K): {K:.3f} rad")
    print(f"Constante de tempo (τ): {tau:.3f} s")

    # 4. Plotar
    plt.plot(times, positions, label='Real')
    plt.plot(times, model(times, K, tau), '--', label='Modelo')
    plt.legend()
    plt.show()

    return K, tau
```

**Interpretação:**
- `K` (ganho): Quanto o motor se move para comando máximo
- `τ` (tau): Quão rápido responde (menor = mais rápido)

---

**Método 2: System ID via Otimização**

Rode simulação com parâmetros variáveis até match com real.

```python
def optimize_simulation_params(real_robot_data, sim_robot):
    """
    Ajusta parâmetros da simulação para match com dados reais
    """
    from scipy.optimize import minimize

    def loss_function(params):
        """
        Roda simulação com params e calcula erro vs real
        """
        mass, friction, inertia = params

        # Atualizar simulação
        sim_robot.set_mass(mass)
        sim_robot.set_friction(friction)
        sim_robot.set_inertia(inertia)

        # Rodar mesmo experimento que no robô real
        sim_trajectory = run_experiment(sim_robot)

        # Calcular distância entre trajetórias
        error = np.mean((sim_trajectory - real_robot_data) ** 2)
        return error

    # Otimizar
    initial_params = [8.0, 0.01, 0.05]  # Valores nominais
    bounds = [(7.0, 9.0), (0.005, 0.02), (0.04, 0.06)]

    result = minimize(
        loss_function,
        initial_params,
        bounds=bounds,
        method='L-BFGS-B'
    )

    optimal_params = result.x
    print(f"Parâmetros identificados: {optimal_params}")

    return optimal_params
```

---

**Método 3: Gray-Box Identification com ML**

Use rede neural para aprender resíduos (diferenças não modeladas).

```python
import torch
import torch.nn as nn

class ResidualDynamicsModel(nn.Module):
    """
    Aprende a diferença entre modelo nominal e real
    """
    def __init__(self, state_dim, action_dim):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim + action_dim, 128),
            nn.ReLU(),
            nn.Linear(128, 128),
            nn.ReLU(),
            nn.Linear(128, state_dim)
        )

    def forward(self, state, action):
        x = torch.cat([state, action], dim=-1)
        residual = self.net(x)
        return residual

# Treinar com dados do robô real
model = ResidualDynamicsModel(state_dim=18, action_dim=12)

for epoch in range(100):
    for state, action, next_state in real_robot_dataset:
        # Predição do modelo nominal (simulação)
        nominal_next = simulate_step(state, action)

        # Residual real
        true_residual = next_state - nominal_next

        # Predição do modelo ML
        pred_residual = model(state, action)

        # Loss
        loss = nn.MSELoss()(pred_residual, true_residual)
        loss.backward()
        optimizer.step()

# Agora, durante simulação:
# next_state = simulate_step(state, action) + model(state, action)
```

---

### 4. Fine-tuning On-Robot (Safety First!)

#### 4.1 Por que Fine-tunar?

Mesmo com domain randomization perfeito, ajustes finais no robô real melhoram performance em 10-30%.

**Mas cuidado:** Treinar no robô real pode quebrá-lo!

---

#### 4.2 Protocolo de Segurança

**Antes de ligar o robô:**

```markdown
CHECKLIST DE SEGURANÇA

Hardware:
- [ ] Área de 3m x 3m livre de obstáculos
- [ ] Piso antiderrapante ou tapete
- [ ] Almofadas/colchões ao redor (amortecer quedas)
- [ ] Botão de emergência acessível (< 2m de distância)
- [ ] Bateria carregada mas não ao máximo (80% ideal)
- [ ] Inspeção visual: parafusos apertados, cabos firmes

Software:
- [ ] Limites de torque configurados (50% do máximo)
- [ ] Watchdog timer ativo (desliga após 100ms sem comando)
- [ ] Kill switch via rede (SSH remoto)
- [ ] Logging ativo (registrar tudo!)
- [ ] Backup da política anterior (rollback fácil)

Equipe:
- [ ] 2+ pessoas presentes (1 no computador, 1 pronta pra segurar robô)
- [ ] Todos usam óculos de proteção
- [ ] Celular carregado (ligar pra suporte se necessário)
- [ ] Plano de emergência definido
```

---

#### 4.3 Estratégias de Fine-tuning Seguro

**Estratégia 1: Inicialização Segura (Safe Exploration)**

Comece de poses conservadoras, aumente risco gradualmente.

```python
class SafeExploration:
    def __init__(self, robot):
        self.robot = robot
        self.episode_count = 0

    def get_initial_pose(self):
        """Pose inicial depende do progresso"""
        if self.episode_count < 10:
            # Muito seguro: joelhos flexionados, centro de massa baixo
            return "crouch_pose"
        elif self.episode_count < 50:
            # Moderado: em pé mas joelhos levemente flexionados
            return "standing_safe"
        else:
            # Normal: pode começar de qualquer pose
            return "random_pose"

    def scale_actions(self, action):
        """Escala ações agressivas no início"""
        if self.episode_count < 20:
            # Primeiros episódios: max 30% de torque
            scale = 0.3
        elif self.episode_count < 100:
            # Episódios iniciais: max 60%
            scale = 0.6
        else:
            # Após 100 episódios: sem limitações
            scale = 1.0

        return action * scale
```

---

**Estratégia 2: Human-in-the-Loop**

Humano pode intervir e corrigir o robô.

```python
class HumanSupervisor:
    def __init__(self, robot):
        self.robot = robot
        self.intervention_count = 0

    def should_intervene(self, state):
        """Detecta situações perigosas"""

        # 1. Vai cair?
        if abs(state['base_orientation_x']) > 0.3:  # > 17 graus
            return True, "Inclinação excessiva"

        # 2. Torques muito altos?
        if any(state['joint_efforts'] > 20.0):  # Nm
            return True, "Torque perigoso"

        # 3. Velocidade angular alta?
        if np.linalg.norm(state['angular_velocity']) > 3.0:  # rad/s
            return True, "Girando rápido demais"

        return False, None

    def intervene(self):
        """Para robô e registra intervenção"""
        print("🚨 INTERVENÇÃO HUMANA!")
        self.robot.emergency_stop()
        self.intervention_count += 1

        # Registrar para aprendizado
        self.log_intervention(state, action, reason)

        # Humano corrige manualmente
        print("Mova o robô para pose segura e pressione ENTER")
        input()

    def collect_correction(self):
        """Registra correção humana como dado de treino"""
        # Estado perigoso -> Ação humana (correção)
        # Isso vira dado de treino supervisionado
        return (state_before, human_action)

# Loop de treino
supervisor = HumanSupervisor(robot)

while True:
    state = robot.get_state()

    # Predição da política
    action = policy.predict(state)

    # Humano pode intervir
    should_stop, reason = supervisor.should_intervene(state)
    if should_stop:
        supervisor.intervene()
        continue

    # Executa ação
    robot.step(action)
```

---

**Estratégia 3: Sim-to-Real Gradual**

Treina em simulações cada vez mais realistas.

```python
# Nível 0: Simulação limpa
train_policy(sim_clean, episodes=1000)

# Nível 1: Simulação com leve randomização
train_policy(sim_light_random, episodes=500, init_from_previous=True)

# Nível 2: Simulação com randomização agressiva
train_policy(sim_heavy_random, episodes=500, init_from_previous=True)

# Nível 3: Simulação com parâmetros identificados do robô real
train_policy(sim_matched_to_real, episodes=200, init_from_previous=True)

# Nível 4: Fine-tuning no robô real (poucos episódios!)
train_policy(real_robot, episodes=50, init_from_previous=True, safe_mode=True)
```

---

### 5. Deployment em Produção

#### 5.1 Arquitetura de Software

**Stack típico para robô em produção:**

```
┌─────────────────────────────────────────┐
│         APLICAÇÃO / UI                  │  ← Dashboard, API REST
├─────────────────────────────────────────┤
│      ORQUESTRAÇÃO (ROS 2)              │  ← Nodes, topics, services
├──────────────┬──────────────────────────┤
│ PERCEPÇÃO   │  CONTROLE   │  NAVEGAÇÃO │
│ (Visão, IA) │  (Motores)  │  (SLAM)    │
├──────────────┴──────────────────────────┤
│         MIDDLEWARE (DDS)                │  ← Comunicação entre nodes
├─────────────────────────────────────────┤
│      DRIVERS DE HARDWARE                │  ← Interface com sensores/atuadores
├─────────────────────────────────────────┤
│         HARDWARE FÍSICO                 │  ← Robô Bumi, Unitree, etc
└─────────────────────────────────────────┘
```

---

#### 5.2 Containerização com Docker

**Por que usar Docker?**
- ✅ Ambiente consistente (dev = prod)
- ✅ Fácil atualização remota
- ✅ Rollback rápido se algo quebrar
- ✅ Isola dependências (não quebra sistema)

**Dockerfile exemplo:**

```dockerfile
# Base: ROS 2 Humble + CUDA (para IA)
FROM nvidia/cuda:11.8.0-cudnn8-runtime-ubuntu22.04

# Instalar ROS 2
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add -
RUN echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    python3-pip

# Instalar dependências Python
COPY requirements.txt /tmp/
RUN pip3 install -r /tmp/requirements.txt

# Copiar código da aplicação
WORKDIR /app
COPY src/ /app/src/
COPY launch/ /app/launch/
COPY config/ /app/config/

# Variáveis de ambiente
ENV ROS_DOMAIN_ID=42
ENV PYTHONUNBUFFERED=1

# Source ROS 2
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Entry point
CMD ["ros2", "launch", "bumi_control", "full_stack.launch.py"]
```

**requirements.txt:**

```txt
torch==2.0.1
torchvision==0.15.2
opencv-python==4.8.0.74
numpy==1.24.3
scipy==1.11.1
matplotlib==3.7.2
pandas==2.0.3
```

---

**Build e Deploy:**

```bash
# 1. Build imagem
docker build -t bumi-control:v1.2.0 .

# 2. Testar localmente
docker run --rm --gpus all \
  -v /dev:/dev \
  --privileged \
  bumi-control:v1.2.0

# 3. Push para registry (se deploy remoto)
docker tag bumi-control:v1.2.0 registry.empresa.com/bumi-control:v1.2.0
docker push registry.empresa.com/bumi-control:v1.2.0

# 4. No robô real (via SSH)
ssh bumi@192.168.1.100
docker pull registry.empresa.com/bumi-control:v1.2.0
docker run -d --restart unless-stopped \
  --name bumi-prod \
  --gpus all \
  -v /dev:/dev \
  --privileged \
  registry.empresa.com/bumi-control:v1.2.0
```

---

#### 5.3 Monitoramento e Logging

**O que monitorar:**

```python
import logging
from prometheus_client import Counter, Histogram, Gauge

# Métricas Prometheus
steps_executed = Counter('robot_steps_total', 'Total steps executed')
action_latency = Histogram('robot_action_latency_seconds', 'Action processing time')
battery_level = Gauge('robot_battery_percent', 'Battery level')
temperature = Gauge('robot_motor_temperature_celsius', 'Motor temperature')
falls_detected = Counter('robot_falls_total', 'Number of falls')

class RobotMonitoring:
    def __init__(self, robot):
        self.robot = robot
        self.logger = logging.getLogger('bumi')

    def log_step(self, state, action, reward):
        """Loga cada passo de controle"""
        steps_executed.inc()

        # Log estruturado
        self.logger.info({
            'timestamp': time.time(),
            'state': {
                'base_position': state['base_pos'].tolist(),
                'base_orientation': state['base_ori'].tolist(),
                'joint_positions': state['joint_pos'].tolist(),
                'battery': state['battery']
            },
            'action': action.tolist(),
            'reward': reward
        })

        # Alertas
        if state['battery'] < 20:
            self.logger.warning(f"⚠️ Bateria baixa: {state['battery']}%")
            battery_level.set(state['battery'])

        for i, temp in enumerate(state['motor_temps']):
            if temp > 70:
                self.logger.error(f"🔥 Motor {i} superaquecendo: {temp}°C")
                temperature.set(temp)

    def detect_fall(self, state):
        """Detecta quedas"""
        roll, pitch, _ = state['base_orientation']
        if abs(roll) > 1.0 or abs(pitch) > 1.0:  # > 57 graus
            falls_detected.inc()
            self.logger.error("🚨 QUEDA DETECTADA!")
            self.robot.emergency_stop()
            self.send_alert_to_team()
            return True
        return False
```

**Dashboard (Grafana):**

```yaml
# grafana-dashboard.json (simplificado)
{
  "panels": [
    {
      "title": "Battery Level",
      "targets": [{"expr": "robot_battery_percent"}],
      "type": "gauge",
      "alert": {
        "conditions": [{"value": 20, "operator": "<"}]
      }
    },
    {
      "title": "Action Latency (p95)",
      "targets": [{"expr": "histogram_quantile(0.95, robot_action_latency_seconds)"}],
      "type": "graph"
    },
    {
      "title": "Falls (last 24h)",
      "targets": [{"expr": "increase(robot_falls_total[24h])"}],
      "type": "stat"
    }
  ]
}
```

---

#### 5.4 Atualizações Over-The-Air (OTA)

**Desafio:** Como atualizar software em 100 robôs em campo?

**Solução:**

```python
# Servidor central
class OTAManager:
    def __init__(self):
        self.fleet = {}  # {robot_id: status}

    def push_update(self, version, target_robots='all'):
        """
        Envia nova versão para robôs
        """
        update_package = {
            'version': version,
            'docker_image': f'registry/bumi-control:{version}',
            'checksum': self.compute_checksum(version),
            'rollback_enabled': True
        }

        if target_robots == 'all':
            robots = self.fleet.keys()
        else:
            robots = target_robots

        # Estratégia: canary deployment
        # 1. Atualiza 5% primeiro
        canary_robots = robots[:len(robots)//20]
        self.deploy_to_robots(canary_robots, update_package)

        # 2. Monitora por 1 hora
        time.sleep(3600)
        if self.check_health(canary_robots):
            # 3. Se OK, atualiza resto
            remaining = robots[len(canary_robots):]
            self.deploy_to_robots(remaining, update_package)
        else:
            # Rollback canary
            self.rollback(canary_robots)

    def deploy_to_robots(self, robots, package):
        """Deploy gradual"""
        for robot_id in robots:
            try:
                # SSH para o robô
                ssh = self.connect_ssh(robot_id)

                # Pull nova imagem Docker
                ssh.exec(f"docker pull {package['docker_image']}")

                # Parar container antigo
                ssh.exec("docker stop bumi-prod")

                # Iniciar novo (com health check)
                ssh.exec(f"docker run -d --name bumi-prod-new {package['docker_image']}")

                # Se health check passa em 60s, remove antigo
                if self.wait_for_health(robot_id, timeout=60):
                    ssh.exec("docker rm bumi-prod")
                    ssh.exec("docker rename bumi-prod-new bumi-prod")
                    self.fleet[robot_id]['version'] = package['version']
                else:
                    # Rollback
                    ssh.exec("docker stop bumi-prod-new && docker rm bumi-prod-new")
                    ssh.exec("docker start bumi-prod")
                    raise Exception("Health check failed")

            except Exception as e:
                logging.error(f"Deploy falhou em {robot_id}: {e}")
                self.fleet[robot_id]['status'] = 'failed'
```

---

### 6. Troubleshooting Comum

#### 6.1 Problema: "Robô cai imediatamente"

**Diagnóstico:**

```python
# Verificar se simulação está muito otimista
print("Centro de massa (sim):", sim_robot.center_of_mass)
print("Centro de massa (real):", real_robot.center_of_mass)

# Diferença > 5cm? Problema!
```

**Soluções:**
1. Refazer system identification (massa real pode estar errada)
2. Aumentar domain randomization em massa e geometria
3. Treinar política com "peso extra" na simulação

---

#### 6.2 Problema: "Ações têm delay visível"

**Diagnóstico:**

```python
import time

def measure_latency(robot):
    latencies = []
    for _ in range(100):
        t0 = time.time()
        robot.set_joint_effort("left_knee", 1.0)
        t1 = time.time()
        latencies.append((t1 - t0) * 1000)  # ms

    print(f"Latência média: {np.mean(latencies):.1f} ms")
    print(f"Latência p95: {np.percentile(latencies, 95):.1f} ms")
```

**Soluções:**
1. Se latência > 50ms: problema de rede ou CPU
   - Usar conexão cabeada (não Wi-Fi)
   - Rodar inferência on-device (não em servidor remoto)
2. Se latência variável: adicionar prediction buffer
   - Prever ação para T+2 (compensa delay)

---

#### 6.3 Problema: "Funciona 5 min depois para"

**Diagnóstico:**

```python
# Checar logs
docker logs bumi-prod --tail 100

# Comum:
# - "CUDA out of memory" → Modelo de IA muito grande
# - "CPU temperature: 95°C" → Superaquecimento
# - "Battery critical" → Bateria acabou
```

**Soluções:**
1. Memory leak → Adicionar `torch.cuda.empty_cache()` periodicamente
2. Superaquecimento → Reduzir frequência de controle (100Hz → 50Hz)
3. Bateria → Adicionar auto-docking para recarga

---

## 🛠️ Atividades Práticas

### Exercício 1: Domain Randomization

**Tempo:** 90 minutos

**Tarefa:** Implementar domain randomization completo no Isaac Sim.

**Código starter:**

```python
# Seu objetivo: preencher essa classe
class MyDomainRandomizer:
    def randomize_all(self, world):
        # TODO: randomizar física
        # TODO: randomizar visual
        # TODO: randomizar sensores
        pass

# Teste
for episode in range(100):
    randomizer.randomize_all(world)
    # Verificar: episódios parecem diferentes?
    # Tirar screenshot de 10 episódios
```

**Entrega:**
- Código Python completo
- 10 screenshots mostrando diversidade
- Relatório: quais parâmetros randomizou e por quê?

📤 [Upload: Exercício 1](../projetos/ex1-upload)

---

### Exercício 2: System Identification

**Tempo:** 60 minutos

**Tarefa:** Identificar parâmetros de um motor simulado.

Você vai receber um "robô caixa-preta" (simulado). Seu trabalho:
1. Aplicar comandos de teste
2. Registrar respostas
3. Estimar: ganho, constante de tempo, fricção

**Código starter fornecido:** `system_id_starter.py`

**Critério de sucesso:**
- Erro < 5% vs parâmetros reais (que serão revelados depois)

📤 [Upload: Exercício 2](../projetos/ex2-upload)

---

### Exercício 3: Deployment com Docker

**Tempo:** 120 minutos

**Tarefa:** Criar Dockerfile para sua aplicação de controle.

**Requisitos:**
- Base: ROS 2 Humble
- Incluir sua política treinada (arquivo .pth)
- Incluir script de launch
- Testar localmente

**Entrega:**
- `Dockerfile`
- `requirements.txt`
- `launch/control.launch.py`
- Screenshot do container rodando

📤 [Upload: Exercício 3](../projetos/ex3-upload)

---

## 📊 Projeto do Módulo: Sim-to-Real Transfer Completo

**Objetivo:** Transferir uma política de caminhada da simulação para um robô "real" (ou simulado com parâmetros realistas).

### Especificações

**Parte 1: Treino com Domain Randomization**
- Treinar política de caminhada em simulação limpa (baseline)
- Adicionar domain randomization agressivo
- Retreinar (pode usar transfer learning)
- Comparar performance: com vs sem randomization

**Parte 2: System Identification**
- Identificar pelo menos 3 parâmetros do robô-alvo
- Ajustar simulação para match
- Treinar política na simulação ajustada

**Parte 3: Teste no Robô-Alvo**
- Deploy cada política (baseline, randomized, finetuned)
- Medir taxa de sucesso em 20 episódios cada
- Analisar: qual foi melhor? Por quê?

**Parte 4: Relatório**
- 4-6 páginas
- Gráficos comparativos
- Vídeos das tentativas
- Lições aprendidas

### Critérios de Avaliação

| Critério | Peso | Descrição |
|----------|------|-----------|
| Domain Randomization | 25% | Implementação completa e justificada |
| System Identification | 20% | Precisão dos parâmetros identificados |
| Resultados experimentais | 30% | Taxa de sucesso, análise rigorosa |
| Relatório | 25% | Clareza, insights, qualidade técnica |

### Entrega

📤 [Upload: Projeto Final Módulo 4.2](../projetos/projeto-m2-upload)

**Prazo:** Até final do Módulo 4.2

---

## 📚 Recursos Complementares

### Papers Fundamentais

📄 **Sim-to-Real Transfer:**
1. "Sim-to-Real Transfer of Robotic Control with Dynamics Randomization" (OpenAI, 2017)
2. "Sim-to-Real: Learning Agile Locomotion For Quadruped Robots" (Google, 2018)
3. "Learning to Walk in Minutes Using Massively Parallel Deep RL" (NVIDIA, 2022)

### Tutoriais e Cursos

🎥 **Vídeos:**
- "Sim2Real in 30 Minutes" - Lex Fridman
- "Domain Randomization Explained" - Two Minute Papers
- "ROS 2 + Docker Best Practices" - The Construct

📚 **Livros (capítulos específicos):**
- "Learning from Randomness" - Cap. 7 de "Reinforcement Learning" (Sutton & Barto)

### Ferramentas

🛠️ **Software:**
- **ROS 2 Humble** - Middleware de robótica
- **Docker + NVIDIA Container Toolkit** - Containerização com GPU
- **Prometheus + Grafana** - Monitoramento
- **Weights & Biases** - Tracking de experimentos

📥 **Código de exemplo:**
- [Domain Randomization no Isaac Sim](https://github.com/NVIDIA/IsaacGymEnvs)
- [System ID com PyTorch](https://github.com/locuslab/lcp-physics)

---

## 🎯 Checklist de Conclusão

Antes de avançar para o Módulo 4.3, certifique-se:

- [ ] Entendo o conceito de "domain gap"
- [ ] Implementei domain randomization em pelo menos 3 dimensões
- [ ] Executei system identification em um robô
- [ ] Criei um Dockerfile funcional
- [ ] Entendo protocolo de segurança para teste em hardware
- [ ] Concluí o projeto de Sim-to-Real Transfer
- [ ] Li pelo menos 1 paper sobre Sim2Real
- [ ] Testei fine-tuning (mesmo que em simulação)

---

## 💬 Discussão e Comunidade

**Participe das discussões:**

💭 **Fórum do Módulo:**
- Compartilhe seus resultados de Sim2Real
- Discuta estratégias de domain randomization
- Tire dúvidas sobre deployment

🎤 **Live Semanal (Terça 20h):**
- Demo de transferências bem-sucedidas
- Q&A com especialistas em Sim2Real
- Troubleshooting coletivo

[→ Entrar na Comunidade Discord](https://discord.gg/fth-nivel4)

---

## ➡️ Próximo Módulo

**Parabéns! Seu robô agora funciona no mundo real!**

No Módulo 4.3, você vai aprender a criar protótipos completos e aplicações reais prontas para o mercado.

**No Módulo 4.3 você vai aprender:**
- Design de produto para robótica
- Integração de sistemas complexos
- Testes de campo e validação
- Documentação profissional
- Apresentação para stakeholders

[→ Iniciar Módulo 4.3: Protótipos e Aplicações]({{ '/niveis/nivel-4/modulo-3' | relative_url }}){: .btn .btn-primary}

---

**Última atualização:** 2025-10-29
**Autor:** Equipe FTH
**Revisores:** 2 engenheiros de robótica + 1 especialista em Sim2Real (ex-Boston Dynamics)
