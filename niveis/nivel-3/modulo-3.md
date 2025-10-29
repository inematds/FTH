---
layout: page
title: "Módulo 3.3: Large Behavior Models (LBMs)"
permalink: /niveis/nivel-3/modulo-3/
---

# 🤖 Módulo 3.3: Large Behavior Models (LBMs)

**O futuro da robótica: modelos fundacionais que generalizam!**

---

## 📋 Informações do Módulo

| Informação | Detalhes |
|------------|----------|
| **Duração estimada** | 20-25 horas |
| **Nível** | Avançado |
| **Pré-requisitos** | RL, Visão Computacional, PyTorch |
| **Ferramentas** | PyTorch, HuggingFace, OpenVLA, ROS 2 |

---

## 🎯 Objetivos de Aprendizado

Ao completar este módulo, você será capaz de:

- [ ] Compreender arquiteturas de transformers para robótica
- [ ] Utilizar modelos pré-treinados (RT-1, RT-2, OpenVLA)
- [ ] Realizar fine-tuning para tarefas específicas
- [ ] Implementar imitation learning avançado
- [ ] Avaliar zero-shot e few-shot generalization
- [ ] Entender trade-offs e limitações de LBMs
- [ ] Aplicar considerações éticas em IA robótica

---

## 📚 Conteúdo Teórico

### 1. O que são Large Behavior Models?

#### Analogia: De LLMs para LBMs

Você conhece **Large Language Models** (GPT, Claude, etc.):
- Treinados em texto massivo da internet
- Generalizam para tarefas nunca vistas
- Few-shot learning com prompts

**Large Behavior Models** fazem o mesmo, mas para **ações robóticas**:
- Treinados em milhões de demonstrações robóticas
- Generalizam para novos objetos/ambientes
- Few-shot learning com demonstrações visuais

#### Diferença de Paradigma

**RL Tradicional:**
```
Treinar política do zero para cada tarefa
→ 10M steps de simulação por tarefa
→ Não generaliza para novos objetos
```

**LBM:**
```
Pré-treinar em 100k+ tarefas variadas
→ Fine-tune com 100-1000 demos na sua tarefa
→ Generaliza para variações não vistas
```

#### Arquitetura Conceitual

```
Observação Visual → Vision Encoder (ViT) →
                      Transformer Decoder →
                      Action Tokens → Ações do Robô
```

**Comparação com LLMs:**

| Aspecto | LLM | LBM |
|---------|-----|-----|
| **Input** | Texto | Imagens + estado |
| **Output** | Texto (tokens) | Ações (contínuas) |
| **Pré-treino** | Internet text | Demonstrações robóticas |
| **Arquitetura** | Transformer | Vision Transformer + Transformer |
| **Tamanho** | 1B-100B params | 50M-10B params |

---

### 2. Estado da Arte em LBMs (2023-2025)

#### RT-1 (Robotics Transformer 1) - Google DeepMind

**Paper:** [RT-1: Robotics Transformer for Real-World Control at Scale (2022)](https://arxiv.org/abs/2212.06817)

**Arquitetura:**
- **Vision Encoder**: EfficientNet + FiLM layers
- **Transformer**: 8 layers, 256M parâmetros
- **Action Head**: Tokeniza ações como vocabulário

**Dataset:**
- 130k demonstrações de robôs
- 700+ tarefas em ambientes reais
- Manipulação de ~5000 objetos

**Resultado chave:**
- 97% sucesso em tarefas vistas
- 76% sucesso em objetos novos (zero-shot)

```python
# Conceitual: Como RT-1 processa uma tarefa
def rt1_forward(image, instruction):
    """
    image: (H, W, 3) - câmera do robô
    instruction: "Pick up the red cup"
    """
    # 1. Vision encoding
    visual_features = efficientnet(image)  # (C, H', W')

    # 2. Language encoding
    language_emb = language_encoder(instruction)  # (D,)

    # 3. FiLM conditioning (modula visão com linguagem)
    conditioned_features = film_layer(visual_features, language_emb)

    # 4. Transformer
    tokens = tokenize(conditioned_features)  # Sequência de tokens
    transformer_output = transformer(tokens)

    # 5. Decode ações
    action_logits = action_head(transformer_output)
    actions = detokenize(action_logits)  # [x, y, z, roll, pitch, yaw, gripper]

    return actions
```

#### RT-2 (Robotics Transformer 2) - Google DeepMind

**Paper:** [RT-2: Vision-Language-Action Models (2023)](https://arxiv.org/abs/2307.15818)

**Inovação:** Usa Vision-Language Models (VLMs) pré-treinados como backbone.

**Arquitetura:**
- Backbone: PaLM-E ou PaLI (modelos VLM massivos)
- Fine-tuning: Apenas cabeça de ação
- Parâmetros: 5B-55B

**Resultado chave:**
- Raciocínio emergente: "Pick up the extinct animal" → pega dinossauro de brinquedo
- 90% sucesso em zero-shot para tarefas semânticas

**Por que funciona melhor:**
- Aproveita conhecimento de internet (VLM)
- Entende semântica ("fruit", "tool", "container")
- Precisa de menos dados robóticos

#### OpenVLA (Open Vision-Language-Action)

**Paper:** [OpenVLA: An Open-Source Vision-Language-Action Model (2024)](https://arxiv.org/abs/2406.09246)

**Diferencial:** Primeiro LBM verdadeiramente open-source e reproduzível.

**Características:**
- Baseado em Llama 2 + CLIP
- Treinado em Open X-Embodiment dataset (1M+ demos)
- 7B parâmetros
- Fine-tuning eficiente com LoRA

```bash
# Instalação
pip install openvla
```

```python
from openvla import OpenVLA
import torch
from PIL import Image

# Carregar modelo pré-treinado
model = OpenVLA.from_pretrained("openvla/openvla-7b")
model.eval()

# Inferência
image = Image.open("robot_view.jpg")
instruction = "Pick up the blue block"

with torch.no_grad():
    action = model.predict_action(
        image=image,
        instruction=instruction,
        unnormalize=True
    )

print(f"Predicted action: {action}")
# Output: [x, y, z, roll, pitch, yaw, gripper]
```

#### Outros Modelos Importantes

**1. Octo (UC Berkeley)**
- Modelo de difusão para ações
- 25M parâmetros
- Especializado em manipulação fina

**2. Pi0 (Physical Intelligence)**
- Multimodal (visão + tato + áudio)
- 3B parâmetros
- Maior performance em dextrous manipulation

**3. RoboCat (Google DeepMind)**
- Self-improvement loop
- Aprende novas tarefas online

---

### 3. Como LBMs São Treinados

#### Etapa 1: Coleta de Dados em Escala

**Open X-Embodiment Dataset:**
- 1M+ trajetórias
- 22 tipos de robôs diferentes
- 1000+ tarefas
- Múltiplos ambientes (labs, casas, fábricas)

**Formato dos dados:**
```python
{
    'observation': {
        'image': np.array([H, W, 3]),  # RGB
        'wrist_image': np.array([H, W, 3]),  # Câmera no pulso
        'state': np.array([7]),  # Joint positions
    },
    'action': np.array([7]),  # [x, y, z, roll, pitch, yaw, gripper]
    'language_instruction': "Pick the red cube",
    'task_id': 'pick_cube_v1',
    'robot_type': 'franka_panda'
}
```

#### Etapa 2: Pré-Treinamento (Pretraining)

**Objetivo:** Aprender representações gerais de manipulação.

**Loss function:**
```python
def behavior_cloning_loss(predicted_action, expert_action):
    """
    Imitation learning: minimize diferença entre ação prevista e demonstração
    """
    return F.mse_loss(predicted_action, expert_action)

def action_diffusion_loss(model, batch):
    """
    Alternativa: modelar distribuição de ações com difusão
    """
    # Adiciona ruído à ação
    noise = torch.randn_like(batch['action'])
    t = torch.randint(0, num_diffusion_steps, (batch_size,))
    noisy_action = add_noise(batch['action'], noise, t)

    # Modelo aprende a remover ruído
    predicted_noise = model(batch['observation'], noisy_action, t)
    return F.mse_loss(predicted_noise, noise)
```

**Treinamento distribuído:**
```python
# Pseudo-código de treinamento
import torch
import torch.distributed as dist
from torch.nn.parallel import DistributedDataParallel as DDP

def train_lbm(model, dataset, num_gpus=8):
    # Setup distribuído
    dist.init_process_group("nccl")
    model = DDP(model)

    optimizer = torch.optim.AdamW(model.parameters(), lr=1e-4)
    dataloader = get_distributed_dataloader(dataset, num_gpus)

    for epoch in range(100):
        for batch in dataloader:
            # Forward
            predicted_actions = model(
                images=batch['observation']['image'],
                instructions=batch['language_instruction']
            )

            # Loss
            loss = F.mse_loss(predicted_actions, batch['action'])

            # Backward
            optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            optimizer.step()

            if step % 100 == 0:
                print(f"Epoch {epoch}, Loss: {loss.item():.4f}")

    # Salvar checkpoint
    torch.save(model.state_dict(), 'lbm_pretrained.pt')
```

#### Etapa 3: Fine-Tuning (Task-Specific)

**Você tem:** Modelo pré-treinado + 100-1000 demos da sua tarefa

**Métodos eficientes:**

**1. LoRA (Low-Rank Adaptation)**

```python
from peft import LoraConfig, get_peft_model

# Configurar LoRA
lora_config = LoraConfig(
    r=16,  # Rank
    lora_alpha=32,
    target_modules=["q_proj", "v_proj"],  # Quais camadas
    lora_dropout=0.1,
    bias="none"
)

# Aplicar ao modelo
model = OpenVLA.from_pretrained("openvla/openvla-7b")
model = get_peft_model(model, lora_config)

# Apenas ~1% dos parâmetros são treináveis!
trainable_params = sum(p.numel() for p in model.parameters() if p.requires_grad)
print(f"Trainable params: {trainable_params:,}")  # ~10M ao invés de 7B
```

**2. Fine-tuning do zero**

```python
def finetune_lbm(pretrained_model, task_dataset, num_epochs=50):
    """
    Fine-tune modelo pré-treinado em tarefa específica
    """
    optimizer = torch.optim.AdamW(model.parameters(), lr=1e-5)  # LR menor!

    for epoch in range(num_epochs):
        for batch in task_dataset:
            predicted_actions = pretrained_model(
                batch['observation'],
                batch['instruction']
            )

            loss = F.mse_loss(predicted_actions, batch['action'])

            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

        # Avaliar a cada epoch
        success_rate = evaluate_model(pretrained_model, eval_env)
        print(f"Epoch {epoch}: Success rate = {success_rate:.2%}")

    return pretrained_model
```

---

### 4. Imitation Learning Avançado

#### Behavioral Cloning (BC)

**O mais simples:** Supervised learning de demonstrações.

```python
import torch
import torch.nn as nn

class BCPolicy(nn.Module):
    def __init__(self, obs_dim, action_dim):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, action_dim)
        )

    def forward(self, obs):
        return self.net(obs)

# Treinar
policy = BCPolicy(obs_dim=84, action_dim=7)
optimizer = torch.optim.Adam(policy.parameters(), lr=1e-3)

for epoch in range(100):
    for obs, action in demo_dataset:
        pred_action = policy(obs)
        loss = F.mse_loss(pred_action, action)

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
```

**Problema do BC:** Distribution shift (vê estados na demo que nunca vê em execução).

#### DAgger (Dataset Aggregation)

**Solução:** Iterar entre coletar dados com política atual e re-treinar.

```python
def dagger(expert_policy, initial_dataset, env, num_iterations=10):
    """
    DAgger: Reduz distribution shift
    """
    policy = train_bc(initial_dataset)
    aggregated_dataset = initial_dataset.copy()

    for iteration in range(num_iterations):
        # Coletar rollouts com política atual
        observations = []
        obs, _ = env.reset()

        for t in range(max_steps):
            observations.append(obs)
            action = policy.predict(obs)
            obs, reward, done, truncated, info = env.step(action)
            if done or truncated:
                break

        # Rotular com política expert
        expert_actions = [expert_policy(obs) for obs in observations]

        # Adicionar ao dataset
        aggregated_dataset.extend(zip(observations, expert_actions))

        # Re-treinar
        policy = train_bc(aggregated_dataset)

    return policy
```

#### Behavior Transformers

**Ideia:** Modelar demonstrações como sequências (como em NLP).

```python
import torch
import torch.nn as nn

class BehaviorTransformer(nn.Module):
    def __init__(self, obs_dim, action_dim, hidden_dim=256, num_layers=6):
        super().__init__()

        self.obs_embedding = nn.Linear(obs_dim, hidden_dim)
        self.action_embedding = nn.Linear(action_dim, hidden_dim)

        # Transformer decoder
        decoder_layer = nn.TransformerDecoderLayer(
            d_model=hidden_dim,
            nhead=8,
            dim_feedforward=1024
        )
        self.transformer = nn.TransformerDecoder(decoder_layer, num_layers)

        self.action_head = nn.Linear(hidden_dim, action_dim)

    def forward(self, obs_seq, action_seq):
        """
        obs_seq: (batch, seq_len, obs_dim)
        action_seq: (batch, seq_len, action_dim)
        """
        # Embeddings
        obs_emb = self.obs_embedding(obs_seq)
        action_emb = self.action_embedding(action_seq)

        # Transformer (auto-regressive)
        output = self.transformer(
            tgt=action_emb.transpose(0, 1),  # (seq_len, batch, dim)
            memory=obs_emb.transpose(0, 1)
        )

        # Predict next actions
        predicted_actions = self.action_head(output.transpose(0, 1))

        return predicted_actions
```

#### Diffusion Policies

**State-of-the-art:** Modelar distribuição de ações com difusão.

**Vantagem:** Captura multi-modalidade (múltiplas formas corretas de fazer algo).

```python
import torch
import torch.nn as nn

class DiffusionPolicy(nn.Module):
    """
    Usa difusão para gerar ações
    Baseado em: Diffusion Policy (Chi et al., 2023)
    """
    def __init__(self, obs_dim, action_dim, num_diffusion_steps=100):
        super().__init__()
        self.num_steps = num_diffusion_steps

        # U-Net para denoising
        self.unet = UNet1D(
            input_dim=action_dim,
            cond_dim=obs_dim,
            hidden_dim=256
        )

        # Noise schedule
        self.register_buffer('betas', self.cosine_beta_schedule())
        self.register_buffer('alphas', 1 - self.betas)
        self.register_buffer('alphas_cumprod', torch.cumprod(self.alphas, dim=0))

    def cosine_beta_schedule(self):
        """Schedule de ruído"""
        steps = self.num_steps
        s = 0.008
        x = torch.linspace(0, steps, steps + 1)
        alphas_cumprod = torch.cos(((x / steps) + s) / (1 + s) * torch.pi * 0.5) ** 2
        alphas_cumprod = alphas_cumprod / alphas_cumprod[0]
        betas = 1 - (alphas_cumprod[1:] / alphas_cumprod[:-1])
        return torch.clip(betas, 0.0001, 0.9999)

    def forward(self, obs, action, t):
        """Denoise step"""
        return self.unet(action, t, cond=obs)

    def sample(self, obs, num_samples=1):
        """Gera ações por difusão reversa"""
        device = obs.device
        batch_size = obs.shape[0]

        # Começar com ruído puro
        action = torch.randn(batch_size, num_samples, action_dim).to(device)

        # Difusão reversa
        for t in reversed(range(self.num_steps)):
            t_batch = torch.full((batch_size,), t, device=device, dtype=torch.long)

            # Prever ruído
            predicted_noise = self.forward(obs, action, t_batch)

            # Remover ruído (DDPM sampling)
            alpha_t = self.alphas[t]
            alpha_cumprod_t = self.alphas_cumprod[t]
            beta_t = self.betas[t]

            action = (action - beta_t / torch.sqrt(1 - alpha_cumprod_t) * predicted_noise) / torch.sqrt(alpha_t)

            # Adicionar ruído (exceto último step)
            if t > 0:
                noise = torch.randn_like(action)
                action += torch.sqrt(beta_t) * noise

        return action
```

---

### 5. Zero-Shot e Few-Shot Generalization

#### O que é Zero-Shot?

**Definição:** Modelo executa tarefa nunca vista, sem fine-tuning.

**Exemplo:**
- Treinado: "Pick red cube", "Pick blue sphere"
- Zero-shot: "Pick yellow pyramid" (nova cor + nova forma)

**Como habilitar:**
- Diversidade no pré-treino
- Linguagem como interface
- Representações generalizáveis

#### Few-Shot Learning

**Definição:** Modelo se adapta com apenas 1-10 demonstrações.

**Técnicas:**

**1. In-Context Learning (como GPT)**

```python
def in_context_predict(model, demo_trajectories, query_observation):
    """
    Modelo vê demonstrações como contexto e prevê ação
    """
    # Concatenar demos + query
    context = []
    for traj in demo_trajectories:
        for obs, action in traj:
            context.append((obs, action))

    context.append((query_observation, None))  # Sem ação

    # Forward pass
    predicted_action = model(context)[-1]  # Última predição

    return predicted_action
```

**2. Meta-Learning (MAML)**

```python
import torch

def maml_few_shot(model, support_set, query_set, inner_lr=0.01, outer_lr=0.001):
    """
    Model-Agnostic Meta-Learning para few-shot
    """
    meta_optimizer = torch.optim.Adam(model.parameters(), lr=outer_lr)

    for episode in range(num_episodes):
        # Inner loop: adapta aos K-shot
        model_copy = copy.deepcopy(model)
        inner_optimizer = torch.optim.SGD(model_copy.parameters(), lr=inner_lr)

        for obs, action in support_set:
            pred = model_copy(obs)
            loss = F.mse_loss(pred, action)
            inner_optimizer.zero_grad()
            loss.backward()
            inner_optimizer.step()

        # Outer loop: avalia em query set
        meta_loss = 0
        for obs, action in query_set:
            pred = model_copy(obs)
            meta_loss += F.mse_loss(pred, action)

        # Update meta-model
        meta_optimizer.zero_grad()
        meta_loss.backward()
        meta_optimizer.step()
```

#### Avaliação de Generalização

**Métricas:**
- **Success rate**: % de tarefas completadas
- **Novel object success**: Sucesso em objetos não vistos
- **Novel scene success**: Sucesso em ambientes novos
- **Long-horizon success**: Sequências de múltiplas tarefas

**Protocolo de teste:**
```python
def evaluate_generalization(model, test_envs):
    results = {
        'seen_objects_seen_scene': [],
        'novel_objects_seen_scene': [],
        'seen_objects_novel_scene': [],
        'novel_objects_novel_scene': []
    }

    for env_type, env in test_envs.items():
        successes = 0
        for episode in range(100):
            obs, _ = env.reset()
            done = False

            while not done:
                action = model.predict(obs)
                obs, reward, done, truncated, info = env.step(action)

            if info['success']:
                successes += 1

        results[env_type] = successes / 100

    return results
```

---

### 6. Implementação Prática com OpenVLA

#### Setup

```bash
# Instalar dependências
pip install openvla torch transformers pillow

# Download modelo (7GB)
python -c "from openvla import OpenVLA; OpenVLA.from_pretrained('openvla/openvla-7b')"
```

#### Inferência Básica

```python
import torch
from openvla import OpenVLA
from PIL import Image
import numpy as np

# Carregar modelo
device = "cuda" if torch.cuda.is_available() else "cpu"
model = OpenVLA.from_pretrained("openvla/openvla-7b").to(device)
model.eval()

# Preparar input
image = Image.open("robot_camera.jpg")
instruction = "Pick up the green bottle and place it in the bin"

# Inferência
with torch.no_grad():
    action = model.predict_action(
        image=image,
        instruction=instruction,
        unnormalize=True  # Desnormaliza para unidades reais
    )

# Ação formato: [x, y, z, roll, pitch, yaw, gripper]
print(f"Predicted action: {action}")
# Ex: [0.45, 0.12, 0.30, 0.0, 3.14, 0.0, 1.0]  # Gripper aberto
```

#### Fine-Tuning em Dataset Customizado

```python
from openvla import OpenVLA, OpenVLAConfig
from openvla.datasets import CustomDataset
from torch.utils.data import DataLoader
import torch

# 1. Preparar dataset
class MyRobotDataset(CustomDataset):
    def __init__(self, data_path):
        self.trajectories = self.load_trajectories(data_path)

    def __len__(self):
        return len(self.trajectories)

    def __getitem__(self, idx):
        traj = self.trajectories[idx]
        return {
            'image': traj['image'],
            'instruction': traj['instruction'],
            'action': traj['action']
        }

dataset = MyRobotDataset('./my_robot_data/')
dataloader = DataLoader(dataset, batch_size=16, shuffle=True)

# 2. Configurar fine-tuning
model = OpenVLA.from_pretrained("openvla/openvla-7b")
model.train()

optimizer = torch.optim.AdamW(model.parameters(), lr=1e-5)

# 3. Loop de treinamento
num_epochs = 10
for epoch in range(num_epochs):
    total_loss = 0

    for batch in dataloader:
        images = batch['image'].to(device)
        instructions = batch['instruction']
        actions = batch['action'].to(device)

        # Forward
        predicted_actions = model(images, instructions)

        # Loss
        loss = torch.nn.functional.mse_loss(predicted_actions, actions)

        # Backward
        optimizer.zero_grad()
        loss.backward()
        torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
        optimizer.step()

        total_loss += loss.item()

    avg_loss = total_loss / len(dataloader)
    print(f"Epoch {epoch+1}/{num_epochs}, Loss: {avg_loss:.4f}")

# 4. Salvar modelo fine-tunado
model.save_pretrained("./my_finetuned_model/")
```

#### Integração com ROS 2

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import torch
from openvla import OpenVLA

class OpenVLARobotNode(Node):
    def __init__(self):
        super().__init__('openvla_robot_node')

        # Carregar modelo
        self.model = OpenVLA.from_pretrained("openvla/openvla-7b")
        self.model.eval()
        self.bridge = CvBridge()

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image', self.image_callback, 10
        )
        self.instruction_sub = self.create_subscription(
            String, '/robot/instruction', self.instruction_callback, 10
        )

        # Publisher
        self.action_pub = self.create_publisher(Twist, '/robot/action', 10)

        self.current_image = None
        self.current_instruction = None

    def image_callback(self, msg):
        self.current_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")

    def instruction_callback(self, msg):
        self.current_instruction = msg.data
        self.predict_and_publish()

    def predict_and_publish(self):
        if self.current_image is None or self.current_instruction is None:
            return

        with torch.no_grad():
            action = self.model.predict_action(
                image=self.current_image,
                instruction=self.current_instruction
            )

        # Converter para mensagem ROS
        twist = Twist()
        twist.linear.x = action[0]
        twist.linear.y = action[1]
        twist.linear.z = action[2]
        twist.angular.x = action[3]
        twist.angular.y = action[4]
        twist.angular.z = action[5]

        self.action_pub.publish(twist)
        self.get_logger().info(f"Published action: {action}")

def main():
    rclpy.init()
    node = OpenVLARobotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

---

### 7. Limitações e Considerações Éticas

#### Limitações Técnicas

**1. Sample Efficiency**
- LBMs precisam de milhares de demonstrações
- Fine-tuning ainda requer 100-1000 demos
- Comparado: humanos aprendem com 1-10 demos

**2. Robustez**
- Falham em situações fora da distribuição
- Sensíveis a mudanças de iluminação/câmera
- Podem gerar ações perigosas se incertos

**3. Interpretabilidade**
- "Black box" - difícil debugar
- Não sabemos o que o modelo "pensa"
- Viés do dataset se propaga

**4. Latência**
- Modelos grandes (7B params) são lentos
- Inferência: 50-200ms por ação
- Trade-off tamanho vs. velocidade

#### Considerações Éticas

**1. Segurança**

```python
def safe_action_wrapper(predicted_action, safety_bounds):
    """
    Clippa ações previstas por limites de segurança
    """
    # Limitar velocidade
    max_velocity = safety_bounds['max_velocity']
    predicted_action[:3] = np.clip(
        predicted_action[:3],
        -max_velocity,
        max_velocity
    )

    # Limitar força do gripper
    max_gripper_force = safety_bounds['max_gripper_force']
    predicted_action[6] = np.clip(predicted_action[6], 0, max_gripper_force)

    # Verificar workspace
    if not is_in_workspace(predicted_action[:3], safety_bounds['workspace']):
        return STOP_ACTION

    return predicted_action
```

**2. Viés e Fairness**

LBMs aprendem vieses dos dados:
- Se demos apenas com objetos de cozinha → pode falhar em ferramentas
- Se demos apenas com mãos humanas brancas → pode não reconhecer outras etnias
- Se demos com gênero específico → pode reforçar estereótipos

**Mitigação:**
- Diversificar datasets de treino
- Auditar performance em subgrupos
- Transparência sobre limitações

**3. Transparência e Responsabilidade**

```python
class ExplainablePolicy:
    """Adiciona explicabilidade a LBMs"""

    def __init__(self, model):
        self.model = model

    def predict_with_explanation(self, obs, instruction):
        # Ação prevista
        action = self.model(obs, instruction)

        # Attention weights (quais partes da imagem influenciaram)
        attention = self.get_attention_map(obs)

        # Confidence score
        confidence = self.estimate_confidence(obs, action)

        # Alternativas (top-k ações)
        alternatives = self.get_alternative_actions(obs, instruction, k=3)

        return {
            'action': action,
            'attention_map': attention,
            'confidence': confidence,
            'alternatives': alternatives,
            'reasoning': self.generate_reasoning(obs, instruction, action)
        }
```

**4. Uso Responsável**

Perguntas críticas:
- Este robô pode causar dano físico/psicológico?
- Substituirá empregos sem alternativas?
- Dados de treino foram obtidos eticamente?
- Sistema tem mecanismo de parada de emergência?
- Usuários entendem limitações?

---

## 💻 Prática Hands-On

### Projeto 1: Fine-Tuning OpenVLA

**Objetivo:** Adaptar OpenVLA para tarefa customizada.

**Etapas:**

1. **Coletar dados (teleoperation)**

```python
import gymnasium as gym
import numpy as np
from PIL import Image

def collect_demonstrations(env, num_episodes=50):
    """Coleta demos com controle humano"""
    dataset = []

    for ep in range(num_episodes):
        obs, _ = env.reset()
        done = False
        trajectory = []

        while not done:
            # Render e mostrar ao operador
            image = env.render()
            cv2.imshow('Robot View', image)

            # Capturar input humano (teclado/joystick)
            action = get_human_action()  # Implementar interface

            # Executar
            next_obs, reward, done, truncated, info = env.step(action)

            # Armazenar
            trajectory.append({
                'image': image,
                'instruction': "Pick the red cube",  # Ou input manual
                'action': action
            })

            obs = next_obs

        dataset.append(trajectory)
        print(f"Collected episode {ep+1}/{num_episodes}")

    return dataset
```

2. **Fine-tuning (código já fornecido acima)**

3. **Avaliar**

```python
def evaluate_finetuned_model(model, env, num_episodes=10):
    successes = 0

    for ep in range(num_episodes):
        obs, _ = env.reset()
        done = False

        while not done:
            image = env.render()
            action = model.predict_action(
                image=image,
                instruction="Pick the red cube"
            )
            obs, reward, done, truncated, info = env.step(action)

        if info['success']:
            successes += 1

    success_rate = successes / num_episodes
    print(f"Success rate: {success_rate:.1%}")
    return success_rate
```

---

### Projeto 2: Zero-Shot Generalization Benchmark

**Objetivo:** Testar generalização do modelo em objetos/tarefas novas.

```python
def zero_shot_benchmark(model):
    """
    Benchmark de generalização zero-shot
    """
    test_scenarios = {
        'seen_object_seen_task': [
            ('pick red cube', 'seen_objects/red_cube.jpg'),
            ('pick blue sphere', 'seen_objects/blue_sphere.jpg')
        ],
        'novel_object_seen_task': [
            ('pick yellow pyramid', 'novel_objects/yellow_pyramid.jpg'),
            ('pick green cylinder', 'novel_objects/green_cylinder.jpg')
        ],
        'seen_object_novel_task': [
            ('stack red cube on blue sphere', 'seen_objects/stack.jpg'),
            ('push blue sphere left', 'seen_objects/push.jpg')
        ],
        'novel_object_novel_task': [
            ('place yellow pyramid in box', 'novel_objects/place.jpg'),
            ('rotate green cylinder', 'novel_objects/rotate.jpg')
        ]
    }

    results = {}

    for category, scenarios in test_scenarios.items():
        successes = 0
        for instruction, image_path in scenarios:
            image = Image.open(image_path)
            action = model.predict_action(image, instruction)

            # Simular ou executar em robô real
            success = execute_and_check(action, instruction)
            if success:
                successes += 1

        results[category] = successes / len(scenarios)
        print(f"{category}: {results[category]:.1%}")

    return results
```

---

### Projeto 3: Multi-Task Learning

**Objetivo:** Treinar um único modelo para múltiplas tarefas.

```python
def train_multitask_lbm(model, task_datasets):
    """
    Treina em múltiplas tarefas simultaneamente
    """
    # Combinar datasets
    combined_dataset = []
    for task_name, dataset in task_datasets.items():
        for sample in dataset:
            sample['task_id'] = task_name
            combined_dataset.append(sample)

    # Embaralhar
    np.random.shuffle(combined_dataset)

    # Treinar
    optimizer = torch.optim.AdamW(model.parameters(), lr=1e-5)

    for epoch in range(num_epochs):
        for batch in DataLoader(combined_dataset, batch_size=16):
            predicted_actions = model(batch['image'], batch['instruction'])
            loss = F.mse_loss(predicted_actions, batch['action'])

            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

    # Avaliar em cada tarefa
    for task_name, task_dataset in task_datasets.items():
        acc = evaluate(model, task_dataset)
        print(f"Task: {task_name}, Accuracy: {acc:.2%}")
```

---

## 🏆 Projeto Final: Sistema LBM Completo

**Objetivo:** Criar pipeline end-to-end de LBM para aplicação real.

**Requisitos:**
1. ✅ Fine-tune modelo pré-treinado (OpenVLA ou similar)
2. ✅ Dataset de 100+ demonstrações em tarefa específica
3. ✅ Zero-shot generalization para 3+ variações
4. ✅ Integração com ROS 2
5. ✅ Taxa de sucesso > 70% em ambiente de teste
6. ✅ Análise de limitações e ética

**Deliverables:**
- Código completo (GitHub)
- Dataset anotado
- Relatório técnico:
  - Arquitetura do modelo
  - Processo de fine-tuning
  - Métricas de performance
  - Análise de generalização
  - Discussão ética
- Vídeo de demonstração (3-5 min)

**Exemplo de estrutura:**

```
lbm-project/
├── data/
│   ├── raw_demos/
│   └── processed/
├── models/
│   ├── pretrained/
│   └── finetuned/
├── src/
│   ├── data_collection.py
│   ├── finetune.py
│   ├── evaluate.py
│   └── ros2_node.py
├── notebooks/
│   └── analysis.ipynb
├── README.md
└── report.pdf
```

---

## 📚 Recursos Adicionais

### Papers Fundamentais

1. **RT-1**: [Robotics Transformer (Brohan et al., 2022)](https://arxiv.org/abs/2212.06817)
2. **RT-2**: [Vision-Language-Action Models (Brohan et al., 2023)](https://arxiv.org/abs/2307.15818)
3. **OpenVLA**: [Open Vision-Language-Action Model (Kim et al., 2024)](https://arxiv.org/abs/2406.09246)
4. **Diffusion Policy**: [Diffusion Policy (Chi et al., 2023)](https://arxiv.org/abs/2303.04137)
5. **Octo**: [Octo (Team et al., 2024)](https://arxiv.org/abs/2405.12213)

### Datasets

- [Open X-Embodiment](https://robotics-transformer-x.github.io/) - 1M+ robot trajectories
- [RoboNet](https://www.robonet.wiki/) - Multi-robot dataset
- [CALVIN](https://github.com/mees/calvin) - Language annotations

### Código e Modelos

```bash
# OpenVLA
pip install openvla

# Octo
pip install octo-models

# Diffusion Policy
git clone https://github.com/real-stanford/diffusion_policy
```

### Comunidades

- [r/MachineLearning](https://reddit.com/r/MachineLearning)
- [Physical Intelligence Discord](https://discord.gg/physicalintelligence)
- [OpenVLA GitHub Discussions](https://github.com/openvla/openvla/discussions)

---

## ✅ Checklist de Conclusão

- [ ] Entendo arquiteturas de transformers para robótica
- [ ] Comparei RT-1, RT-2 e OpenVLA
- [ ] Executei inferência com modelo pré-treinado
- [ ] Coletei dataset de demonstrações
- [ ] Realizei fine-tuning com sucesso
- [ ] Avaliei zero-shot generalization
- [ ] Implementei nó ROS 2 com LBM
- [ ] Analisei limitações e ética
- [ ] Completei projeto final
- [ ] Publiquei código no GitHub

---

## 🎓 Conclusão do Nível 3

Parabéns por completar o Nível 3 - Desenvolvedor! Você agora domina:

✅ Algoritmos avançados de RL (PPO, SAC)
✅ Visão computacional para robótica
✅ Large Behavior Models (LBMs)
✅ State-of-the-art em robótica e IA

**Próximo nível:** Nível 4 - Profissional
- Modelos de negócio
- Sim-to-Real transfer
- Protótipos e aplicações reais

---

## 🚀 Para Onde Ir Agora?

**Caminhos possíveis:**

1. **Pesquisa Acadêmica**: Papers, contribuições open-source
2. **Indústria**: Robótica em empresas (Tesla, Figure, Physical Intelligence)
3. **Startup**: Criar sua própria empresa de robótica
4. **Educação**: Ensinar próxima geração de robóticos

**O futuro da robótica humanoide está em suas mãos!**

---

**Última atualização:** 2025-10-29
**Autor:** Programa FTH
**Licença:** MIT
