---
layout: page
title: Recursos e Ferramentas
permalink: /recursos/
---

# 🛠️ Recursos e Ferramentas

Tudo que você precisa para começar sua jornada em robótica humanoide e IA, **organizado por nível de dificuldade**.

---

## 🌟 Para Iniciantes (Nível 1)

### Simuladores Gratuitos

#### NVIDIA Isaac Sim
- **O que é:** Simulador profissional de robótica com física realista
- **Quando usar:** Projetos que exigem física precisa e gráficos avançados
- **Custo:** Gratuito (requer cadastro NVIDIA)
- **Requisitos:** GPU NVIDIA (GTX 1060+), 8GB RAM
- 📥 [Download Isaac Sim](https://developer.nvidia.com/isaac-sim)
- 📖 [Tutorial de Instalação]({{ '/tutoriais/isaac-sim-setup' | relative_url }})

#### Webots
- **O que é:** Simulador open-source leve e versátil
- **Quando usar:** Ideal para começar, funciona em qualquer PC
- **Custo:** Totalmente gratuito
- **Requisitos:** 4GB RAM, qualquer GPU básica
- 📥 [Download Webots](https://cyberbotics.com)
- 📖 [Tutorial de Instalação]({{ '/tutoriais/webots-setup' | relative_url }})

#### Gazebo Classic
- **O que é:** Simulador padrão da comunidade ROS
- **Quando usar:** Integração com ROS 2 e projetos comunitários
- **Custo:** Gratuito
- **Requisitos:** Linux preferível, 8GB RAM
- 📥 [Download Gazebo](https://gazebosim.org)

#### MuJoCo
- **O que é:** Motor de física rápido para RL
- **Quando usar:** Treinamento massivo de RL (milhares de robôs)
- **Custo:** Gratuito (agora open-source)
- **Requisitos:** 4GB RAM, Python
- 📥 [Download MuJoCo](https://mujoco.org)

---

## ⚡ Para Intermediários (Nível 2)

### Python e Bibliotecas

#### Python 3.10+
- **Instalação:** [python.org/downloads](https://python.org/downloads)
- **Pacotes essenciais:**
```bash
pip install numpy scipy matplotlib
pip install opencv-python
pip install rospy
```

#### ROS 2 (Robot Operating System)
- **O que é:** Framework padrão para programação de robôs
- **Versão recomendada:** ROS 2 Humble (LTS)
- **Sistema:** Ubuntu 22.04 (ou WSL no Windows)
- 📥 [Instalação ROS 2](https://docs.ros.org/en/humble/Installation.html)
- 📖 [Tutorial FTH de ROS 2]({{ '/tutoriais/ros2-basico' | relative_url }})

#### Jupyter Notebooks
- **Para quê:** Experimentação e aprendizado interativo
```bash
pip install jupyter notebook
```

---

## 🔥 Para Avançados (Nível 3)

### Machine Learning e RL

#### PyTorch
- **O que é:** Framework principal para Deep Learning
- **Quando usar:** Treinar redes neurais e algoritmos de RL
```bash
pip install torch torchvision
```
- 📖 [Tutorial PyTorch para Robótica]({{ '/tutoriais/pytorch-rl' | relative_url }})

#### Stable-Baselines3
- **O que é:** Biblioteca com algoritmos de RL prontos (PPO, SAC, etc)
```bash
pip install stable-baselines3[extra]
```

#### Gymnasium
- **O que é:** API padrão para ambientes de RL (sucessor do OpenAI Gym)
```bash
pip install gymnasium
```

### Visão Computacional

#### OpenCV
- **Instalação:**
```bash
pip install opencv-python opencv-contrib-python
```
- 📖 [Tutorial de Visão para Robôs]({{ '/tutoriais/visao-computacional' | relative_url }})

#### YOLO v8
- **Para quê:** Detecção de objetos em tempo real
```bash
pip install ultralytics
```

#### RoboFlow
- **O que é:** Plataforma para criar datasets de visão
- **Custo:** Gratuito para projetos pequenos
- 📥 [roboflow.com](https://roboflow.com)

---

## 👑 Para Profissionais (Nível 4)

### Hardware

#### Noetix Bumi
- **O que é:** Robô humanoide educacional brasileiro
- **Custo:** A partir de R$ 1.500 (kit básico)
- **Onde comprar:** [noetix.ai](https://noetix.ai)
- **Specs:**
  - 30cm altura
  - 12 graus de liberdade
  - Câmera RGB
  - IMU e sensores de distância
  - Programável em Python + ROS 2

#### Unitree R1
- **O que é:** Robô humanoide profissional chinês
- **Custo:** ~R$ 25.000
- **Specs:** 1.2m, 40 DoF, RL-ready

#### Raspberry Pi 4
- **Para quê:** Cérebro de robôs DIY
- **Custo:** ~R$ 400-600
- **Onde comprar:** Importadores oficiais ou FilipeFlop

#### Arduino + Servo Motores
- **Para quê:** Prototipagem rápida de baixo custo
- **Custo:** Kit completo ~R$ 200-400

---

## 📚 Materiais de Estudo

### Documentação Oficial

- 📖 [ROS 2 Documentation](https://docs.ros.org)
- 📖 [PyTorch Tutorials](https://pytorch.org/tutorials)
- 📖 [Isaac Sim Docs](https://docs.omniverse.nvidia.com/isaacsim)
- 📖 [Stable-Baselines3 Docs](https://stable-baselines3.readthedocs.io)

### Cursos Online Complementares

#### Gratuitos
- 🎓 [CS50 - Introduction to Computer Science (Harvard)](https://cs50.harvard.edu)
- 🎓 [Deep Reinforcement Learning (UC Berkeley)](http://rail.eecs.berkeley.edu/deeprlcourse/)
- 🎓 [ROS 2 Tutorial Playlist (YouTube)](https://youtube.com)

#### Pagos
- 🎓 Coursera: "Robotics Specialization" (University of Pennsylvania)
- 🎓 Udacity: "Robotics Software Engineer Nanodegree"

---

## 🎥 Canais e Comunidades

### YouTube

- 🎬 **Boston Dynamics** - Robôs em ação
- 🎬 **Two Minute Papers** - Papers de IA explicados
- 🎬 **Lex Fridman** - Entrevistas com pesquisadores
- 🎬 **The Construct** - Tutoriais de ROS

### Comunidades

#### Discord FTH (Oficial)
- 💬 Tire dúvidas em tempo real
- 🤝 Colabore em projetos
- 📢 Fique por dentro de eventos
- 🔗 [Entrar no Discord](https://discord.gg/fth-brasil)

#### Reddit
- r/robotics
- r/reinforcementlearning
- r/MachineLearning

#### GitHub
- 🌟 [Awesome Robotics](https://github.com/kiloreux/awesome-robotics)
- 🌟 [Awesome RL](https://github.com/aikorea/awesome-rl)

---

## 📊 Datasets e Benchmarks

### Datasets de Movimento Humanoide

- **CMU Motion Capture Database** - Movimentos humanos capturados
- **HumanML3D** - Dados de locomoção e gestos
- **AMASS** - Archive of Motion Capture as Surface Shapes

### Benchmarks de RL

- **MuJoCo Humanoid-v4** - Locomoção bípede
- **PyBullet Humanoid** - Open-source alternativo
- **Isaac Gym Humanoid** - GPU-acelerado

---

## 🖥️ Ambientes de Desenvolvimento

### IDEs Recomendados

#### VS Code
- **Por quê:** Leve, extensível, ótimo para Python
- 📥 [Download VS Code](https://code.visualstudio.com)
- **Extensões essenciais:**
  - Python (Microsoft)
  - Pylance
  - ROS (Microsoft)
  - Jupyter

#### PyCharm
- **Por quê:** IDE completa para Python
- 📥 [Download PyCharm](https://jetbrains.com/pycharm)
- **Versão:** Community (gratuita) é suficiente

### Terminal e Bash

#### Windows
- **WSL 2** (Windows Subsystem for Linux) - Essencial para ROS 2
- 📖 [Tutorial de Instalação WSL]({{ '/tutoriais/wsl-setup' | relative_url }})

#### Linux
- Ubuntu 22.04 LTS (recomendado)

#### macOS
- Terminal nativo funciona, mas ROS 2 tem suporte limitado

---

## ☁️ Plataformas Cloud

### Google Colab
- **Para quê:** Treinar redes neurais sem GPU local
- **Custo:** Gratuito (GPU limitada) ou R$ 50/mês (Colab Pro)
- 🔗 [colab.research.google.com](https://colab.research.google.com)

### Kaggle Kernels
- **Para quê:** Datasets e competições
- **Custo:** Gratuito (30h GPU/semana)
- 🔗 [kaggle.com](https://kaggle.com)

---

## 🧰 Kits Educacionais Completos

### Kit Iniciante (~R$ 0)
- ✅ Webots (simulador)
- ✅ Python 3.10
- ✅ Jupyter Notebook
- ✅ OpenCV
- **Computador mínimo:** 4GB RAM, CPU dual-core

### Kit Intermediário (~R$ 0)
- ✅ Isaac Sim ou Gazebo
- ✅ ROS 2 Humble
- ✅ PyTorch + Stable-Baselines3
- **Computador recomendado:** 16GB RAM, GPU NVIDIA GTX 1060+

### Kit Avançado (~R$ 2.000)
- ✅ Tudo acima
- ✅ Raspberry Pi 4 (8GB)
- ✅ Motores servo + sensores
- ✅ Kit estrutural para montar robô DIY

### Kit Profissional (~R$ 10.000+)
- ✅ Noetix Bumi ou similar
- ✅ Workstation com GPU RTX 3080+
- ✅ Licenças e mentorias

---

## 📖 Livros Recomendados

### Robótica
- 📕 "Introduction to Autonomous Mobile Robots" - Siegwart
- 📕 "Robotics: Modelling, Planning and Control" - Siciliano
- 📕 "Modern Robotics" - Lynch & Park (gratuito online)

### Reinforcement Learning
- 📕 "Reinforcement Learning: An Introduction" - Sutton & Barto (gratuito)
- 📕 "Deep Reinforcement Learning Hands-On" - Lapan

### Python
- 📕 "Python Crash Course" - Eric Matthes
- 📕 "Automate the Boring Stuff with Python" - Al Sweigart (gratuito)

---

## 🆘 Suporte e Ajuda

### FAQ Técnico
- 🔧 [Problemas Comuns de Instalação]({{ '/faq-tecnico' | relative_url }})
- 🔧 [Erros de Simulador]({{ '/faq-simuladores' | relative_url }})
- 🔧 [Dúvidas de ROS 2]({{ '/faq-ros' | relative_url }})

### Fóruns
- Stack Overflow (tag: `robotics`, `ros2`, `reinforcement-learning`)
- ROS Answers (answers.ros.org)
- Robotics Stack Exchange

### Contato Direto
- 📧 Email: **suporte@fth.edu.br** (fictício)
- 💬 Discord: Canal #ajuda-tecnica

---

## 🎁 Recursos Exclusivos FTH

### Templates de Código
- 🐍 [Repositório GitHub FTH](https://github.com/inematds/FTH)
  - Código dos projetos de cada nível
  - Ambientes de RL personalizados
  - Scripts de setup automatizado

### Tutoriais em Vídeo
- 🎥 Playlist completa no YouTube
- 🎥 Lives semanais de dúvidas

### Webinars
- 📅 Mensal: Convidados da indústria
- 📅 Quinzenal: Showcase de projetos de alunos

---

## 🔄 Atualizações

Esta página é atualizada **mensalmente** com novos recursos, ferramentas e recomendações da comunidade.

**Última atualização:** 2025-10-29

---

<div class="cta-section">
  <h2>Pronto para Começar?</h2>
  <p>Agora que você conhece as ferramentas, é hora de colocar a mão na massa!</p>
  <a href="{{ '/niveis/nivel-1' | relative_url }}" class="btn btn-large btn-primary">Iniciar Nível 1 →</a>
</div>
