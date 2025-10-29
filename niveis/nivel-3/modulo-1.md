---
layout: page
title: "Módulo 3.1: Algoritmos de IA Avançados (PPO, SAC)"
permalink: /niveis/nivel-3/modulo-1/
---

# 🧠 Módulo 3.1: Algoritmos de IA Avançados (PPO, SAC)

**Domine os algoritmos state-of-the-art de Aprendizado por Reforço!**

---

## 📋 Informações do Módulo

| Informação | Detalhes |
|------------|----------|
| **Duração estimada** | 15-20 horas |
| **Nível** | Avançado |
| **Pré-requisitos** | Nível 2 concluído, conhecimento de RL básico |
| **Ferramentas** | Python, Stable-Baselines3, PyTorch, TensorBoard |

---

## 🎯 Objetivos de Aprendizado

Ao completar este módulo, você será capaz de:

- [ ] Compreender profundamente PPO (Proximal Policy Optimization)
- [ ] Implementar e otimizar SAC (Soft Actor-Critic)
- [ ] Comparar e escolher algoritmos apropriados para diferentes tarefas
- [ ] Realizar tuning avançado de hiperparâmetros
- [ ] Treinar políticas com ambientes vetorizados
- [ ] Monitorar e debugar treinamento com TensorBoard
- [ ] Analisar convergência e performance

---

## 📚 Conteúdo Teórico

### 1. Revisão: Fundamentos de Reinforcement Learning

Antes de mergulharmos nos algoritmos avançados, vamos revisar conceitos essenciais:

#### Estado, Ação e Recompensa

```python
# Estrutura básica de um ambiente de RL
class RobotEnvironment:
    def __init__(self):
        self.state = None

    def reset(self):
        """Retorna estado inicial"""
        return initial_state

    def step(self, action):
        """Executa ação e retorna (próximo_estado, recompensa, done, info)"""
        next_state = self.transition(self.state, action)
        reward = self.compute_reward(next_state)
        done = self.is_terminal(next_state)
        return next_state, reward, done, {}
```

#### Política vs. Value Function

- **Política (π)**: Mapeia estados para ações
  - Estocástica: π(a|s) = probabilidade de ação a dado estado s
  - Determinística: a = π(s)

- **Value Function (V)**: Valor esperado de um estado
  - V^π(s) = E[∑γ^t * r_t | s_0=s, π]

- **Q-Function**: Valor esperado de ação em estado
  - Q^π(s,a) = E[∑γ^t * r_t | s_0=s, a_0=a, π]

#### Exploration vs. Exploitation

**Problema fundamental do RL**: balancear exploração (tentar coisas novas) vs. exploração (usar o que já funciona).

**Estratégias comuns:**
- ε-greedy (usado em DQN)
- Entropy regularization (usado em SAC)
- Noise no espaço de ações (usado em DDPG)

#### Reward Shaping

Projetar boas funções de recompensa é **crítico** para o sucesso do RL.

**Princípios:**
- **Densas**: Feedback frequente
- **Normalizadas**: Recompensas na mesma escala
- **Sem contradições**: Não incentivar comportamentos opostos
- **Alinhadas**: Refletir objetivo real

**Exemplo prático - Robô andando:**

```python
def compute_reward_walking(state):
    """Função de recompensa bem projetada para caminhar"""

    # 1. Recompensa principal: velocidade para frente
    forward_velocity = state['base_velocity_x']
    reward = forward_velocity * 1.0  # Peso principal

    # 2. Penalidade por queda
    base_height = state['base_height']
    if base_height < 0.3:  # Muito baixo = queda
        reward -= 10.0

    # 3. Incentivo por manter corpo reto
    body_orientation = state['base_roll']  # Ângulo de inclinação
    reward -= abs(body_orientation) * 0.5

    # 4. Penalidade leve por uso de energia
    torques = state['joint_torques']
    energy_cost = np.sum(np.square(torques)) * 0.001
    reward -= energy_cost

    # 5. Bonus por estabilidade
    if state['steps_without_falling'] > 100:
        reward += 2.0

    return reward
```

**Armadilhas comuns:**
- Recompensa muito esparsa (robô nunca aprende)
- Recompensa densa demais (overfitting a comportamentos locais)
- Recompensas conflitantes (ex: maximizar velocidade E minimizar movimento)

---

### 2. PPO (Proximal Policy Optimization)

#### O que é PPO?

**PPO** é o algoritmo de RL mais popular para robótica, desenvolvido pela OpenAI em 2017. Ele resolve um problema fundamental: como atualizar a política de forma **eficiente** sem causar mudanças **drásticas** que desestabilizem o treinamento.

**Paper original:** [Proximal Policy Optimization Algorithms (Schulman et al., 2017)](https://arxiv.org/abs/1707.06347)

#### Intuição: Por que PPO?

Imagine que você está treinando um robô para andar:
- **Iteração 1**: Robô aprende a dar pequenos passos
- **Iteração 2**: Você atualiza a política de forma agressiva
- **Resultado**: Robô esquece tudo e volta a cair

**Problema**: Policy Gradient tradicional pode fazer updates muito grandes, destruindo o que foi aprendido.

**Solução do PPO**: Limitar o tamanho do update para garantir melhorias incrementais e estáveis.

#### Matemática do PPO (Simplificada)

O PPO otimiza uma função objetivo que **limita** quanto a nova política pode diferir da antiga:

```
L^CLIP(θ) = E[min(r_t(θ) * A_t, clip(r_t(θ), 1-ε, 1+ε) * A_t)]

Onde:
- r_t(θ) = π_θ(a|s) / π_θ_old(a|s)  [razão entre política nova e antiga]
- A_t = vantagem (quão melhor que média é tomar ação a)
- ε = limite de clipping (tipicamente 0.2)
```

**Interpretação:**
- Se `r_t` estiver entre `[1-ε, 1+ε]`, aceita o update
- Se `r_t` tentar ir muito além, **clippa** (limita) o valor

Isso garante que a nova política não se afasta muito da antiga.

#### Componentes do PPO

**1. Actor-Critic Architecture**

```python
import torch
import torch.nn as nn

class PPOActorCritic(nn.Module):
    def __init__(self, obs_dim, action_dim):
        super().__init__()

        # Feature extractor compartilhado
        self.shared = nn.Sequential(
            nn.Linear(obs_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU()
        )

        # Actor: gera distribuição de ações
        self.actor_mean = nn.Linear(256, action_dim)
        self.actor_logstd = nn.Parameter(torch.zeros(action_dim))

        # Critic: estima value function
        self.critic = nn.Linear(256, 1)

    def forward(self, obs):
        features = self.shared(obs)

        # Política (Gaussian para ações contínuas)
        action_mean = self.actor_mean(features)
        action_std = torch.exp(self.actor_logstd)

        # Value function
        value = self.critic(features)

        return action_mean, action_std, value

    def get_action(self, obs, deterministic=False):
        """Amostra ação da política"""
        action_mean, action_std, value = self.forward(obs)

        if deterministic:
            return action_mean, value

        # Amostra de distribuição Gaussian
        dist = torch.distributions.Normal(action_mean, action_std)
        action = dist.sample()
        log_prob = dist.log_prob(action).sum(dim=-1)

        return action, log_prob, value
```

**2. Generalized Advantage Estimation (GAE)**

GAE é usado para calcular vantagens de forma mais estável:

```python
def compute_gae(rewards, values, dones, gamma=0.99, lambda_=0.95):
    """
    Calcula Generalized Advantage Estimation

    GAE balanceia bias vs. variance no cálculo de vantagens:
    - λ=0: baixa variance, alto bias (TD(0))
    - λ=1: alta variance, baixo bias (Monte Carlo)
    - λ=0.95: sweet spot empírico
    """
    advantages = []
    gae = 0

    for t in reversed(range(len(rewards))):
        if t == len(rewards) - 1:
            next_value = 0
        else:
            next_value = values[t + 1]

        # TD error: δ = r + γV(s') - V(s)
        delta = rewards[t] + gamma * next_value * (1 - dones[t]) - values[t]

        # GAE: A = δ + γλδ' + (γλ)²δ'' + ...
        gae = delta + gamma * lambda_ * (1 - dones[t]) * gae
        advantages.insert(0, gae)

    return torch.tensor(advantages)
```

**3. PPO Update Loop**

```python
def ppo_update(policy, optimizer, states, actions, old_log_probs,
               advantages, returns, clip_epsilon=0.2, epochs=10):
    """
    Atualiza política usando PPO
    """
    for epoch in range(epochs):
        # Forward pass
        action_mean, action_std, values = policy(states)
        dist = torch.distributions.Normal(action_mean, action_std)
        new_log_probs = dist.log_prob(actions).sum(dim=-1)

        # Ratio r_t = π_new / π_old
        ratio = torch.exp(new_log_probs - old_log_probs)

        # PPO clipped objective
        surr1 = ratio * advantages
        surr2 = torch.clamp(ratio, 1-clip_epsilon, 1+clip_epsilon) * advantages
        actor_loss = -torch.min(surr1, surr2).mean()

        # Value function loss
        value_loss = F.mse_loss(values.squeeze(), returns)

        # Entropy bonus (incentiva exploração)
        entropy = dist.entropy().mean()

        # Loss total
        loss = actor_loss + 0.5 * value_loss - 0.01 * entropy

        # Backward pass
        optimizer.zero_grad()
        loss.backward()
        nn.utils.clip_grad_norm_(policy.parameters(), max_norm=0.5)
        optimizer.step()
```

#### Implementação Completa com Stable-Baselines3

```python
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv
from stable_baselines3.common.callbacks import EvalCallback
import gymnasium as gym

# 1. Criar ambiente
def make_env():
    def _init():
        env = gym.make('HumanoidStandup-v4')
        return env
    return _init

# 2. Ambientes vetorizados (paralelo)
n_envs = 8
env = SubprocVecEnv([make_env() for _ in range(n_envs)])

# 3. Configurar PPO
model = PPO(
    policy="MlpPolicy",
    env=env,

    # Hiperparâmetros core
    learning_rate=3e-4,
    n_steps=2048,        # Steps por rollout por env
    batch_size=64,       # Minibatch size
    n_epochs=10,         # Epochs por update
    gamma=0.99,          # Discount factor
    gae_lambda=0.95,     # GAE lambda

    # PPO específico
    clip_range=0.2,      # Epsilon de clipping
    clip_range_vf=None,  # Clipping no value function

    # Regularização
    ent_coef=0.01,       # Entropy bonus
    vf_coef=0.5,         # Value function coef
    max_grad_norm=0.5,   # Gradient clipping

    # Network
    policy_kwargs=dict(
        net_arch=[dict(pi=[256, 256], vf=[256, 256])]
    ),

    # Logging
    verbose=1,
    tensorboard_log="./ppo_humanoid_tensorboard/"
)

# 4. Callback para avaliação
eval_callback = EvalCallback(
    eval_env=gym.make('HumanoidStandup-v4'),
    best_model_save_path='./best_model/',
    log_path='./eval_logs/',
    eval_freq=10000,
    deterministic=True,
    render=False
)

# 5. Treinar
model.learn(
    total_timesteps=10_000_000,  # 10M steps
    callback=eval_callback
)

# 6. Salvar
model.save("ppo_humanoid_final")

# 7. Testar
obs, info = env.reset()
for _ in range(1000):
    action, _states = model.predict(obs, deterministic=True)
    obs, rewards, dones, truncated, infos = env.step(action)
    env.render()
```

#### Quando Usar PPO?

**PPO é ideal para:**
- ✅ Tarefas de locomoção (andar, correr, pular)
- ✅ Manipulação robótica
- ✅ Controle contínuo multi-dimensional
- ✅ Quando você precisa de estabilidade no treinamento
- ✅ Quando você tem muitos cores (paraleliza bem)

**PPO não é ideal para:**
- ❌ Tarefas que exigem exploração agressiva
- ❌ Ambientes muito esparsos
- ❌ Quando você precisa de sample efficiency extrema

---

### 3. SAC (Soft Actor-Critic)

#### O que é SAC?

**SAC** (Soft Actor-Critic) é um algoritmo de RL off-policy que maximiza tanto a recompensa quanto a **entropia** da política. Foi desenvolvido pela UC Berkeley em 2018.

**Paper original:** [Soft Actor-Critic: Off-Policy Maximum Entropy Deep RL (Haarnoja et al., 2018)](https://arxiv.org/abs/1801.01290)

#### Intuição: Maximum Entropy RL

SAC resolve um problema diferente do PPO:

**Objetivo tradicional de RL:**
```
Maximize: E[∑ γ^t * r_t]
```

**Objetivo do SAC (Maximum Entropy):**
```
Maximize: E[∑ γ^t * (r_t + α * H(π(·|s_t)))]

Onde H(π) = entropia da política
```

**Por que adicionar entropia?**
1. **Exploração automática**: Política naturalmente explora
2. **Robustez**: Aprende múltiplas soluções
3. **Transfer**: Generaliza melhor para novos ambientes

**Analogia**: Ao invés de aprender "uma única forma certa" de pegar um objeto, o robô aprende "várias formas que funcionam", tornando-o mais adaptável.

#### Arquitetura do SAC

SAC usa **três redes neurais**:

```python
import torch
import torch.nn as nn
import torch.nn.functional as F

class SACActorCritic(nn.Module):
    def __init__(self, obs_dim, action_dim):
        super().__init__()

        # Actor: Política estocástica (Gaussian Squashed)
        self.actor = nn.Sequential(
            nn.Linear(obs_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU()
        )
        self.mean = nn.Linear(256, action_dim)
        self.log_std = nn.Linear(256, action_dim)

        # Critic 1: Q-function
        self.q1 = nn.Sequential(
            nn.Linear(obs_dim + action_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, 1)
        )

        # Critic 2: Q-function (double Q-learning)
        self.q2 = nn.Sequential(
            nn.Linear(obs_dim + action_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, 1)
        )

    def get_action(self, obs, deterministic=False):
        """Amostra ação usando reparameterization trick"""
        features = self.actor(obs)
        mean = self.mean(features)
        log_std = self.log_std(features)
        log_std = torch.clamp(log_std, -20, 2)  # Estabilidade
        std = torch.exp(log_std)

        if deterministic:
            action = mean
        else:
            # Reparameterization trick
            normal = torch.distributions.Normal(mean, std)
            z = normal.rsample()  # Permite backprop
            action = torch.tanh(z)  # Squash para [-1, 1]

        # Log probability (com correção para tanh)
        log_prob = normal.log_prob(z) - torch.log(1 - action.pow(2) + 1e-6)
        log_prob = log_prob.sum(dim=-1, keepdim=True)

        return action, log_prob

    def get_q_values(self, obs, action):
        """Retorna Q-values das duas redes"""
        x = torch.cat([obs, action], dim=-1)
        q1 = self.q1(x)
        q2 = self.q2(x)
        return q1, q2
```

#### Componentes Chave do SAC

**1. Replay Buffer (Experience Replay)**

SAC é **off-policy**: aprende de transições antigas armazenadas:

```python
import numpy as np

class ReplayBuffer:
    def __init__(self, obs_dim, action_dim, capacity=1_000_000):
        self.capacity = capacity
        self.ptr = 0
        self.size = 0

        self.states = np.zeros((capacity, obs_dim), dtype=np.float32)
        self.actions = np.zeros((capacity, action_dim), dtype=np.float32)
        self.rewards = np.zeros((capacity, 1), dtype=np.float32)
        self.next_states = np.zeros((capacity, obs_dim), dtype=np.float32)
        self.dones = np.zeros((capacity, 1), dtype=np.float32)

    def add(self, state, action, reward, next_state, done):
        self.states[self.ptr] = state
        self.actions[self.ptr] = action
        self.rewards[self.ptr] = reward
        self.next_states[self.ptr] = next_state
        self.dones[self.ptr] = done

        self.ptr = (self.ptr + 1) % self.capacity
        self.size = min(self.size + 1, self.capacity)

    def sample(self, batch_size):
        idx = np.random.randint(0, self.size, size=batch_size)
        return (
            torch.FloatTensor(self.states[idx]),
            torch.FloatTensor(self.actions[idx]),
            torch.FloatTensor(self.rewards[idx]),
            torch.FloatTensor(self.next_states[idx]),
            torch.FloatTensor(self.dones[idx])
        )
```

**2. Target Networks**

SAC usa redes target para estabilizar treinamento:

```python
import copy

# Criar redes target
target_critic = copy.deepcopy(critic)

# Soft update (Polyak averaging)
def soft_update(target, source, tau=0.005):
    """
    Target: θ' ← τθ + (1-τ)θ'
    """
    for target_param, param in zip(target.parameters(), source.parameters()):
        target_param.data.copy_(
            tau * param.data + (1.0 - tau) * target_param.data
        )
```

**3. Automatic Entropy Tuning**

SAC ajusta automaticamente o coeficiente de entropia α:

```python
# Entropia alvo (heurística: -dim(A))
target_entropy = -action_dim

# α como variável treinável (em log-space para estabilidade)
log_alpha = torch.zeros(1, requires_grad=True)
alpha_optimizer = torch.optim.Adam([log_alpha], lr=3e-4)

# Atualizar α
def update_alpha(log_prob):
    alpha = log_alpha.exp()
    alpha_loss = -(log_alpha * (log_prob + target_entropy).detach()).mean()

    alpha_optimizer.zero_grad()
    alpha_loss.backward()
    alpha_optimizer.step()

    return alpha.detach()
```

**4. SAC Update Loop**

```python
def sac_update(actor, critic, target_critic, replay_buffer,
               alpha, batch_size=256):
    # Amostra batch
    states, actions, rewards, next_states, dones = replay_buffer.sample(batch_size)

    # --- Atualizar Critic ---
    with torch.no_grad():
        # Amostra próxima ação da política atual
        next_actions, next_log_probs = actor.get_action(next_states)

        # Q-target usando min de duas redes (Clipped Double Q-learning)
        q1_next, q2_next = target_critic.get_q_values(next_states, next_actions)
        q_next = torch.min(q1_next, q2_next) - alpha * next_log_probs
        q_target = rewards + gamma * (1 - dones) * q_next

    # Q atual
    q1, q2 = critic.get_q_values(states, actions)

    # Loss de critic (MSE)
    critic_loss = F.mse_loss(q1, q_target) + F.mse_loss(q2, q_target)

    critic_optimizer.zero_grad()
    critic_loss.backward()
    critic_optimizer.step()

    # --- Atualizar Actor ---
    new_actions, log_probs = actor.get_action(states)
    q1_new, q2_new = critic.get_q_values(states, new_actions)
    q_new = torch.min(q1_new, q2_new)

    # Maximizar Q - entropia (equivalent: minimizar -Q + α*log_prob)
    actor_loss = (alpha * log_probs - q_new).mean()

    actor_optimizer.zero_grad()
    actor_loss.backward()
    actor_optimizer.step()

    # --- Atualizar Alpha ---
    alpha = update_alpha(log_probs.detach())

    # --- Soft update de target networks ---
    soft_update(target_critic, critic, tau=0.005)

    return actor_loss.item(), critic_loss.item()
```

#### Implementação Completa com Stable-Baselines3

```python
from stable_baselines3 import SAC
from stable_baselines3.common.noise import NormalActionNoise
import gymnasium as gym
import numpy as np

# 1. Criar ambiente
env = gym.make('Humanoid-v4')

# 2. Configurar SAC
model = SAC(
    policy="MlpPolicy",
    env=env,

    # Hiperparâmetros core
    learning_rate=3e-4,
    buffer_size=1_000_000,   # Replay buffer size
    learning_starts=10000,   # Steps antes de começar treino
    batch_size=256,          # Batch size
    tau=0.005,               # Soft update coef
    gamma=0.99,              # Discount factor

    # SAC específico
    train_freq=1,            # Atualiza a cada step
    gradient_steps=1,        # Gradient steps por update
    ent_coef='auto',         # Auto-tune entropy coef
    target_entropy='auto',   # -dim(A)

    # Network architecture
    policy_kwargs=dict(
        net_arch=[256, 256],
        log_std_init=-3
    ),

    # Logging
    verbose=1,
    tensorboard_log="./sac_humanoid_tensorboard/"
)

# 3. Treinar
model.learn(
    total_timesteps=3_000_000,
    log_interval=10
)

# 4. Salvar e testar
model.save("sac_humanoid_final")

obs, info = env.reset()
for _ in range(1000):
    action, _states = model.predict(obs, deterministic=True)
    obs, reward, done, truncated, info = env.step(action)
    env.render()
```

#### Quando Usar SAC?

**SAC é ideal para:**
- ✅ Tarefas que exigem **exploração intensa**
- ✅ Manipulação de objetos complexa
- ✅ Ambientes contínuos com recompensas esparsas
- ✅ Quando você precisa de **sample efficiency** (aprende rápido)
- ✅ Quando você quer robustez e generalização

**SAC não é ideal para:**
- ❌ Ações discretas (use DQN ou PPO)
- ❌ Quando você tem muitos cores ociosos (não paraleliza como PPO)
- ❌ Ambientes muito simples (overhead de replay buffer)

---

### 4. PPO vs SAC: Comparação Detalhada

| Aspecto | PPO | SAC |
|---------|-----|-----|
| **Tipo** | On-policy | Off-policy |
| **Paralelização** | Excelente (8-32 envs) | Limitada (1 env) |
| **Sample Efficiency** | Baixa-Média | Alta |
| **Estabilidade** | Muito alta | Alta |
| **Exploração** | ε-decay ou noise | Automatic (entropy) |
| **Memória** | Baixa | Alta (replay buffer) |
| **Velocidade por step** | Rápida | Mais lenta |
| **Hiperparâmetros** | Sensível | Mais robusto |
| **Convergência** | Gradual | Rápida (se convergir) |

**Regra de ouro:**
- **Locomoção/Controle simples**: PPO
- **Manipulação/Exploração**: SAC
- **Muitos cores disponíveis**: PPO
- **Sample efficiency crítica**: SAC
- **Não sabe qual usar**: Comece com PPO

---

### 5. Tuning de Hiperparâmetros

#### Hiperparâmetros Críticos do PPO

**Learning Rate (lr)**
- Range: `1e-5` a `1e-3`
- Padrão: `3e-4`
- **Como ajustar**: Se treinamento oscilar muito, diminua. Se muito lento, aumente.

```python
# Decaying learning rate
from torch.optim.lr_scheduler import LinearLR

scheduler = LinearLR(
    optimizer,
    start_factor=1.0,
    end_factor=0.1,
    total_iters=1000
)
```

**n_steps (Rollout Length)**
- Range: `512` a `4096`
- Padrão: `2048`
- **Trade-off**: Maior = mais estável, mas menos updates

**Clip Range (ε)**
- Range: `0.1` a `0.3`
- Padrão: `0.2`
- **Como ajustar**: Se updates muito conservadores, aumente. Se instável, diminua.

**GAE Lambda (λ)**
- Range: `0.9` a `0.99`
- Padrão: `0.95`
- **Trade-off**: Menor = menos variance, mais bias. Maior = mais variance, menos bias.

#### Hiperparâmetros Críticos do SAC

**Learning Rate**
- Range: `1e-5` a `1e-3`
- Padrão: `3e-4`
- **Dica**: SAC geralmente tolera LR maiores que PPO

**Batch Size**
- Range: `64` a `512`
- Padrão: `256`
- **Trade-off**: Maior = mais estável, mas mais lento

**Buffer Size**
- Range: `100k` a `10M`
- Padrão: `1M`
- **Limite**: RAM disponível (1M steps ≈ 2-4 GB)

**Tau (Soft Update)**
- Range: `0.001` a `0.01`
- Padrão: `0.005`
- **Como ajustar**: Menor = mais estável, mas mais lento

#### Grid Search Automatizado

```python
from stable_baselines3 import PPO
import optuna

def objective(trial):
    # Hiperparâmetros a otimizar
    lr = trial.suggest_loguniform('lr', 1e-5, 1e-3)
    n_steps = trial.suggest_categorical('n_steps', [512, 1024, 2048, 4096])
    clip_range = trial.suggest_uniform('clip_range', 0.1, 0.3)
    gae_lambda = trial.suggest_uniform('gae_lambda', 0.9, 0.99)

    # Criar modelo
    model = PPO(
        'MlpPolicy',
        env='Humanoid-v4',
        learning_rate=lr,
        n_steps=n_steps,
        clip_range=clip_range,
        gae_lambda=gae_lambda,
        verbose=0
    )

    # Treinar por um período curto
    model.learn(total_timesteps=100_000)

    # Avaliar performance
    mean_reward = evaluate_policy(model, env, n_eval_episodes=10)

    return mean_reward

# Otimizar com Optuna
study = optuna.create_study(direction='maximize')
study.optimize(objective, n_trials=50)

print(f"Melhores hiperparâmetros: {study.best_params}")
```

---

## 💻 Prática Hands-On

### Projeto 1: Treinar Humanoid com PPO

**Objetivo:** Treinar um robô humanoide para ficar em pé usando PPO.

```python
# train_humanoid_ppo.py
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv
from stable_baselines3.common.callbacks import CheckpointCallback
import gymnasium as gym

# Criar 8 ambientes paralelos
def make_env():
    def _init():
        return gym.make('HumanoidStandup-v4')
    return _init

env = SubprocVecEnv([make_env() for _ in range(8)])

# Callback para salvar checkpoints
checkpoint_callback = CheckpointCallback(
    save_freq=100_000,
    save_path='./checkpoints/',
    name_prefix='humanoid_ppo'
)

# Configurar PPO otimizado
model = PPO(
    'MlpPolicy',
    env,
    learning_rate=3e-4,
    n_steps=2048,
    batch_size=64,
    n_epochs=10,
    gamma=0.99,
    gae_lambda=0.95,
    clip_range=0.2,
    ent_coef=0.01,
    verbose=1,
    tensorboard_log='./ppo_humanoid/'
)

# Treinar por 10M steps (~3-4 horas em CPU moderno)
model.learn(
    total_timesteps=10_000_000,
    callback=checkpoint_callback
)

model.save('humanoid_ppo_final')
```

**Executar:**
```bash
python train_humanoid_ppo.py

# Monitorar com TensorBoard
tensorboard --logdir ./ppo_humanoid/
```

**Métricas esperadas:**
- Ep Reward Mean: 0 → 5000+ em 5-10M steps
- Ep Length Mean: ~500 (máximo do ambiente)
- Value Loss: Deve estabilizar em ~100-200

---

### Projeto 2: Comparar PPO vs SAC

**Objetivo:** Treinar o mesmo ambiente com ambos e comparar.

```python
# compare_algorithms.py
import gymnasium as gym
from stable_baselines3 import PPO, SAC
import matplotlib.pyplot as plt
import pandas as pd

env_name = 'Ant-v4'  # Formiga quadrúpede

# 1. Treinar PPO
print("Treinando PPO...")
ppo_model = PPO('MlpPolicy', env_name, verbose=0)
ppo_rewards = []

for i in range(100):
    ppo_model.learn(total_timesteps=10_000, reset_num_timesteps=False)
    mean_reward = evaluate_policy(ppo_model, env_name, n_eval_episodes=5)
    ppo_rewards.append(mean_reward)
    print(f"PPO Step {i*10000}: {mean_reward}")

# 2. Treinar SAC
print("Treinando SAC...")
sac_model = SAC('MlpPolicy', env_name, verbose=0)
sac_rewards = []

for i in range(100):
    sac_model.learn(total_timesteps=10_000, reset_num_timesteps=False)
    mean_reward = evaluate_policy(sac_model, env_name, n_eval_episodes=5)
    sac_rewards.append(mean_reward)
    print(f"SAC Step {i*10000}: {mean_reward}")

# 3. Plotar comparação
plt.figure(figsize=(10, 6))
plt.plot(ppo_rewards, label='PPO', linewidth=2)
plt.plot(sac_rewards, label='SAC', linewidth=2)
plt.xlabel('Training Steps (x10k)')
plt.ylabel('Mean Episode Reward')
plt.title(f'PPO vs SAC on {env_name}')
plt.legend()
plt.grid(True)
plt.savefig('ppo_vs_sac_comparison.png')
plt.show()

# 4. Estatísticas finais
print("\n=== Resultados Finais ===")
print(f"PPO Final Reward: {ppo_rewards[-1]:.2f}")
print(f"SAC Final Reward: {sac_rewards[-1]:.2f}")
print(f"PPO converge em ~{len([r for r in ppo_rewards if r > 0.8*max(ppo_rewards)])}k steps")
print(f"SAC converge em ~{len([r for r in sac_rewards if r > 0.8*max(sac_rewards)])}k steps")
```

---

### Projeto 3: Custom Reward Function

**Objetivo:** Criar função de recompensa customizada para tarefa específica.

**Tarefa:** Robô deve pegar um cubo e colocá-lo em uma caixa.

```python
import gymnasium as gym
from gymnasium import spaces
import numpy as np

class PickAndPlaceEnv(gym.Env):
    """Ambiente customizado: pegar e colocar objeto"""

    def __init__(self):
        super().__init__()

        # Estado: [posição_robô(3), vel_robô(3), posição_cubo(3), posição_alvo(3)]
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(12,), dtype=np.float32
        )

        # Ação: [delta_x, delta_y, delta_z, gripper_force]
        self.action_space = spaces.Box(
            low=-1, high=1, shape=(4,), dtype=np.float32
        )

        self.reset()

    def reset(self, seed=None):
        super().reset(seed=seed)

        # Posição inicial aleatória
        self.robot_pos = np.array([0.0, 0.0, 0.5])
        self.robot_vel = np.zeros(3)
        self.cube_pos = np.random.uniform([-0.3, -0.3, 0.0], [0.3, 0.3, 0.0])
        self.target_pos = np.array([0.5, 0.5, 0.2])

        self.cube_grasped = False
        self.steps = 0

        return self._get_obs(), {}

    def _get_obs(self):
        return np.concatenate([
            self.robot_pos,
            self.robot_vel,
            self.cube_pos,
            self.target_pos
        ])

    def step(self, action):
        # Mover robô
        self.robot_pos += action[:3] * 0.05
        self.robot_vel = action[:3]

        # Verificar se pegou cubo
        dist_to_cube = np.linalg.norm(self.robot_pos - self.cube_pos)
        if dist_to_cube < 0.05 and action[3] > 0.5:  # Gripper fechado
            self.cube_grasped = True

        # Se pegou, cubo move com robô
        if self.cube_grasped:
            self.cube_pos = self.robot_pos.copy()

        # Calcular recompensa (aqui está a mágica!)
        reward = self._compute_reward()

        # Termina se alcançou objetivo ou timeout
        dist_to_target = np.linalg.norm(self.cube_pos - self.target_pos)
        done = (dist_to_target < 0.1 and self.cube_grasped) or self.steps > 500

        self.steps += 1

        return self._get_obs(), reward, done, False, {}

    def _compute_reward(self):
        """
        Design de recompensa multi-stage para pick-and-place
        """
        reward = 0.0

        dist_robot_to_cube = np.linalg.norm(self.robot_pos - self.cube_pos)
        dist_cube_to_target = np.linalg.norm(self.cube_pos - self.target_pos)

        # FASE 1: Alcançar cubo
        if not self.cube_grasped:
            # Recompensa densa por aproximar do cubo
            reward += 1.0 / (1.0 + dist_robot_to_cube)

            # Bonus por tocar no cubo
            if dist_robot_to_cube < 0.05:
                reward += 2.0

        # FASE 2: Pegar cubo
        if self.cube_grasped:
            reward += 5.0  # Bonus por pegar (one-time)

            # Recompensa densa por levar ao alvo
            reward += 2.0 / (1.0 + dist_cube_to_target)

            # Bonus enorme por colocar no lugar
            if dist_cube_to_target < 0.1:
                reward += 20.0

        # PENALIDADES
        # - Penalidade leve por movimentos bruscos (energia)
        velocity_penalty = np.sum(np.square(self.robot_vel)) * 0.01
        reward -= velocity_penalty

        # - Penalidade por sair dos limites
        if np.any(np.abs(self.robot_pos[:2]) > 1.0):
            reward -= 1.0

        return reward

# Treinar com PPO
env = PickAndPlaceEnv()
model = PPO('MlpPolicy', env, verbose=1)
model.learn(total_timesteps=500_000)
model.save('pick_and_place_ppo')
```

---

## 📊 Monitoramento com TensorBoard

TensorBoard é essencial para debugar treinamento de RL:

```bash
# Instalar
pip install tensorboard

# Executar
tensorboard --logdir ./logs/
```

**Métricas importantes:**

1. **Episode Reward Mean**: Recompensa média por episódio
   - Deve aumentar consistentemente
   - Se oscila muito: reward shaping ruim ou lr muito alta

2. **Episode Length Mean**: Duração dos episódios
   - Se sempre max_steps: robô não está terminando tarefa
   - Se muito curto: robô está falhando rápido

3. **Policy Loss**: Loss do actor
   - Deve diminuir e estabilizar
   - Se aumenta: treinamento instável

4. **Value Loss**: Loss do critic
   - Deve diminuir
   - Se muito alta: critic não consegue estimar valores

5. **Explained Variance**: Quão bem critic prevê retornos
   - Ideal: > 0.7
   - Baixa: critic ruim, vai prejudicar actor

6. **Entropy**: Aleatoriedade da política
   - Deve diminuir gradualmente
   - Se cai rápido demais: exploração insuficiente

**Exemplo de análise:**

```python
# Carregar logs do TensorBoard
from tensorboard.backend.event_processing import event_accumulator
import matplotlib.pyplot as plt

ea = event_accumulator.EventAccumulator('logs/PPO_1/')
ea.Reload()

# Extrair métricas
rewards = [(s.step, s.value) for s in ea.Scalars('rollout/ep_rew_mean')]
steps, values = zip(*rewards)

# Plotar
plt.figure(figsize=(12, 6))
plt.plot(steps, values)
plt.xlabel('Steps')
plt.ylabel('Mean Episode Reward')
plt.title('Learning Curve')
plt.grid(True)
plt.show()
```

---

## 🏆 Projeto Final

### Sistema de Locomoção Bípede Avançado

**Objetivo:** Treinar um robô humanoide para:
1. Andar para frente
2. Andar para trás
3. Virar esquerda/direita
4. Manter equilíbrio em terrenos irregulares

**Especificações:**
- Ambiente: `Humanoid-v4` ou Isaac Sim
- Algoritmo: PPO ou SAC (sua escolha)
- Critério de sucesso: Andar 10m sem cair

```python
# advanced_humanoid_locomotion.py
import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv, VecNormalize
from stable_baselines3.common.callbacks import EvalCallback, CheckpointCallback

def make_env():
    def _init():
        env = gym.make('Humanoid-v4')
        return env
    return _init

# 1. Criar ambientes paralelos
n_envs = 16
env = SubprocVecEnv([make_env() for _ in range(n_envs)])

# 2. Normalização de observações e recompensas
env = VecNormalize(
    env,
    norm_obs=True,
    norm_reward=True,
    clip_obs=10.0,
    clip_reward=10.0
)

# 3. Callbacks
checkpoint_callback = CheckpointCallback(
    save_freq=50_000,
    save_path='./checkpoints/',
    name_prefix='humanoid_advanced'
)

eval_callback = EvalCallback(
    eval_env=gym.make('Humanoid-v4'),
    best_model_save_path='./best_model/',
    log_path='./eval_logs/',
    eval_freq=20_000,
    deterministic=True
)

# 4. Configurar modelo com hiperparâmetros otimizados
model = PPO(
    'MlpPolicy',
    env,
    learning_rate=3e-4,
    n_steps=2048,
    batch_size=64,
    n_epochs=10,
    gamma=0.99,
    gae_lambda=0.95,
    clip_range=0.2,
    ent_coef=0.005,  # Menos entropia para comportamento mais consistente
    vf_coef=0.5,
    max_grad_norm=0.5,
    policy_kwargs=dict(
        net_arch=[dict(pi=[256, 256], vf=[256, 256])],
        activation_fn=torch.nn.ReLU
    ),
    verbose=1,
    tensorboard_log='./logs/'
)

# 5. Treinar
print("Iniciando treinamento...")
model.learn(
    total_timesteps=20_000_000,  # 20M steps
    callback=[checkpoint_callback, eval_callback]
)

# 6. Salvar modelo final
model.save('humanoid_advanced_final')
env.save('vec_normalize.pkl')

# 7. Testar
print("Testando modelo treinado...")
obs = env.reset()
for _ in range(5000):
    action, _states = model.predict(obs, deterministic=True)
    obs, rewards, dones, info = env.step(action)
    env.render()

print("Treinamento completo!")
```

**Critérios de avaliação:**
- [ ] Robô anda 10m sem cair
- [ ] Velocidade média > 0.5 m/s
- [ ] Mantém equilíbrio (corpo reto)
- [ ] Código documentado e limpo
- [ ] Logs e gráficos de treinamento
- [ ] Análise de performance

---

## 📚 Recursos Adicionais

### Papers Fundamentais

1. **PPO**: [Proximal Policy Optimization Algorithms](https://arxiv.org/abs/1707.06347) - Schulman et al., 2017
2. **SAC**: [Soft Actor-Critic: Off-Policy Maximum Entropy Deep RL](https://arxiv.org/abs/1801.01290) - Haarnoja et al., 2018
3. **GAE**: [High-Dimensional Continuous Control Using Generalized Advantage Estimation](https://arxiv.org/abs/1506.02438) - Schulman et al., 2015

### Tutoriais e Cursos

- [Stable-Baselines3 Documentation](https://stable-baselines3.readthedocs.io/)
- [Spinning Up in Deep RL (OpenAI)](https://spinningup.openai.com/)
- [Deep RL Course (Hugging Face)](https://huggingface.co/deep-rl-course)

### Bibliotecas

```bash
pip install stable-baselines3[extra]
pip install gymnasium[mujoco]
pip install tensorboard
pip install optuna  # Hyperparameter tuning
```

### Comunidades

- [r/reinforcementlearning](https://www.reddit.com/r/reinforcementlearning/)
- [RL Discord](https://discord.gg/xhfNqQv)
- [Stable-Baselines3 Discussions](https://github.com/DLR-RM/stable-baselines3/discussions)

---

## ✅ Checklist de Conclusão

- [ ] Entendo a diferença entre on-policy e off-policy
- [ ] Implementei PPO do zero (ou com SB3)
- [ ] Implementei SAC do zero (ou com SB3)
- [ ] Comparei PPO vs SAC em um ambiente
- [ ] Projetei uma reward function customizada
- [ ] Fiz tuning de hiperparâmetros
- [ ] Treinei com ambientes vetorizados
- [ ] Analisei logs no TensorBoard
- [ ] Completei o projeto final
- [ ] Publiquei código no GitHub

---

## 🚀 Próximos Passos

Parabéns por dominar algoritmos avançados de RL! Agora você está pronto para:

1. **Módulo 3.2**: Visão Computacional para Robótica
2. **Módulo 3.3**: Large Behavior Models (LBMs)
3. **Nível 4**: Aplicações Profissionais

---

**Última atualização:** 2025-10-29
**Autor:** Programa FTH
**Licença:** MIT
