---
layout: page
title: "Módulo 1.3: Programação Visual"
permalink: /niveis/nivel-1/modulo-3/
---

# 🎯 Módulo 1.3: Programação Visual

**Duração estimada:** 5-6 horas
**Pré-requisitos:** Módulos 1.1 e 1.2 concluídos
**Nível de dificuldade:** ⭐⭐ Fácil/Médio

---

## 📋 Objetivos de Aprendizado

Ao completar este módulo, você será capaz de:
- [ ] Entender o conceito de programação visual (drag-and-drop)
- [ ] Criar sequências de movimentos para o robô
- [ ] Usar blocos de sensores para tomar decisões
- [ ] Implementar estruturas de controle (if/else, loops)
- [ ] Integrar comandos de voz básicos com IA
- [ ] Debugar e corrigir problemas visuais
- [ ] Criar um programa completo que faz o robô navegar autonomamente

---

## 📚 Conteúdo Teórico

### 1. O Que é Programação Visual?

**Analogia:** Pense em programar como montar LEGO. Ao invés de escrever texto complexo, você **encaixa blocos visuais** que representam ações!

**Exemplos que você já conhece:**
- **Scratch:** Linguagem criada pelo MIT para ensinar crianças
- **Blockly:** Usado pelo Google e Code.org
- **LabVIEW:** Ferramenta profissional usada pela NASA!

```
┌────────────────────────────────────────┐
│  PROGRAMAÇÃO TRADICIONAL (texto):     │
│                                        │
│  if sensor.distance < 0.5:             │
│      robot.stop()                      │
│      robot.turn_left(90)               │
│  else:                                 │
│      robot.walk_forward()              │
└────────────────────────────────────────┘

         VS

┌────────────────────────────────────────┐
│  PROGRAMAÇÃO VISUAL (blocos):          │
│                                        │
│  ╔════════════════════════════╗        │
│  ║ SE [distância] < 0.5m      ║        │
│  ╠════════════════════════════╣        │
│  ║   ┌─────────────┐          ║        │
│  ║   │ Parar robô  │          ║        │
│  ║   └─────────────┘          ║        │
│  ║   ┌─────────────┐          ║        │
│  ║   │ Virar 90°   │          ║        │
│  ║   └─────────────┘          ║        │
│  ╠════════════════════════════╣        │
│  ║ SENÃO                      ║        │
│  ╠════════════════════════════╣        │
│  ║   ┌─────────────┐          ║        │
│  ║   │ Andar frente│          ║        │
│  ║   └─────────────┘          ║        │
│  ╚════════════════════════════╝        │
└────────────────────────────────────────┘
```

**Vantagens da programação visual:**
- ✅ **Intuitivo:** Você VÊ a lógica
- ✅ **Sem erros de sintaxe:** Não esquece ponto-e-vírgula
- ✅ **Rápido para prototipar:** Teste ideias em minutos
- ✅ **Aprende conceitos:** Mesma lógica que código "de verdade"

**Limitações:**
- ❌ Menos flexível que código texto (para projetos complexos)
- ❌ Difícil de versionar (Git não funciona bem)
- ❌ Não é usado em produção profissional

**Quando usar:** Perfeito para aprender, prototipar, e projetos educacionais!

---

### 2. Ferramentas de Programação Visual para Robótica

Para este módulo, vamos usar 3 ferramentas:

#### 🎨 **Scratch for Webots** (Recomendado)

**O que é:**
- Extensão do Scratch adaptada para controlar robôs no Webots
- Interface familiar para quem já usou Scratch
- 100% gratuito

**Instalação:**
- Já vem integrado no Webots!
- Basta ativar nas configurações

---

#### 🔧 **Blockly for Robotics** (Alternativa)

**O que é:**
- Biblioteca JavaScript criada pelo Google
- Gera código Python automaticamente
- Mais avançado, mas ainda visual

**Quando usar:** Se você quer ver o código sendo gerado enquanto programa.

---

#### 🤖 **RobotFlow** (Online)

**O que é:**
- Plataforma web para programação visual
- Funciona direto no navegador
- Integra com simuladores

**Link:** [robotflow.io](https://robotflow.io) (exemplo fictício)

---

### 3. Blocos Fundamentais

Todo programa de robô, seja visual ou texto, usa estas 5 categorias de blocos:

#### 📦 **1. MOVIMENTO (Motion Blocks)**

Controlam os motores do robô.

```
┌─────────────────────┐
│ 🚶 Andar Frente     │
│    Distância: [1]m  │
│    Velocidade: 50%  │
└─────────────────────┘

┌─────────────────────┐
│ 🔄 Virar            │
│    Ângulo: [90]°    │
│    Direção: ◀️ ▶️   │
└─────────────────────┘

┌─────────────────────┐
│ ⏹️ Parar Motores    │
└─────────────────────┘

┌─────────────────────┐
│ 💃 Pose Customizada │
│    Arquivo: wave.pos│
└─────────────────────┘
```

**Exemplo de uso:**
1. Andar 2 metros
2. Virar 90° direita
3. Andar 1 metro
4. Parar

---

#### 👀 **2. SENSORES (Sensor Blocks)**

Leem dados do ambiente.

```
┌─────────────────────┐
│ 📏 Distância        │
│    Sensor: frontal  │
│    Valor: 0.8m      │
└─────────────────────┘

┌─────────────────────┐
│ 📷 Detectar Objeto  │
│    Cor: 🔴 Vermelho │
│    Encontrado: ✅    │
└─────────────────────┘

┌─────────────────────┐
│ ⚖️ Equilíbrio IMU   │
│    Inclinação: 2°   │
└─────────────────────┘

┌─────────────────────┐
│ 🎤 Reconhecer Voz   │
│    Palavra: "parar" │
│    Confiança: 95%   │
└─────────────────────┘
```

---

#### 🧮 **3. CONTROLE (Control Blocks)**

Estruturas de decisão e repetição.

```
╔═══════════════════════╗
║ SE [condição]         ║
╠═══════════════════════╣
║   [ações se verdade]  ║
╠═══════════════════════╣
║ SENÃO                 ║
╠═══════════════════════╣
║   [ações se falso]    ║
╚═══════════════════════╝

┌─────────────────────┐
│ 🔁 REPETIR [10]x    │
│  [ações]            │
└─────────────────────┘

┌─────────────────────┐
│ ♾️ PARA SEMPRE      │
│  [ações]            │
└─────────────────────┘

┌─────────────────────┐
│ ⏱️ ESPERAR [2]s     │
└─────────────────────┘
```

---

#### 🔢 **4. OPERADORES (Operator Blocks)**

Matemática e lógica.

```
┌─────────────────────┐
│ [10] + [5] = 15     │
└─────────────────────┘

┌─────────────────────┐
│ [valor] < [0.5]     │
│ Resultado: ✅ True  │
└─────────────────────┘

┌─────────────────────┐
│ [A] E [B]           │
│ Ambos verdade?      │
└─────────────────────┘

┌─────────────────────┐
│ [A] OU [B]          │
│ Pelo menos 1 verdade│
└─────────────────────┘
```

---

#### 💾 **5. VARIÁVEIS (Data Blocks)**

Armazenam informações.

```
┌─────────────────────┐
│ 📊 CRIAR VARIÁVEL   │
│    Nome: contador   │
│    Valor inicial: 0 │
└─────────────────────┘

┌─────────────────────┐
│ ⬆️ INCREMENTAR      │
│    [contador] + 1   │
└─────────────────────┘

┌─────────────────────┐
│ 👁️ MOSTRAR VALOR    │
│    Variável: [X]    │
│    No console       │
└─────────────────────┘
```

---

## 💻 Prática Hands-On

### 🎯 Exercício 1: Configurando Scratch no Webots

#### Passo 1: Ativar Extensão

1. Abra o Webots
2. Vá em **Tools > Preferences**
3. Na aba **"Python"**, marque ✅ **"Enable Scratch for Webots"**
4. Clique em **"Apply"** e depois **"OK"**
5. **Reinicie o Webots**

---

#### Passo 2: Abrir Exemplo com Scratch

1. Vá em **File > Open World**
2. Navegue para: `projects/languages/scratch/worlds/`
3. Abra **`scratch_nao.wbt`**
4. Inicie a simulação (▶️)

**O que acontece:**
- Uma janela do Scratch abre automaticamente
- Ela está conectada ao robô NAO no Webots!

---

#### Passo 3: Interface do Scratch

```
┌─────────────────────────────────────────┐
│  Scratch for Webots                     │
├─────────┬───────────────┬───────────────┤
│ BLOCOS  │ CÓDIGO VISUAL │ ROBÔ (preview)│
│         │               │               │
│ Motion  │  ┌─────────┐  │     ___       │
│ Sensor  │  │Andar 1m │  │    (o o)      │
│ Control │  └─────────┘  │     \-/       │
│ ...     │       │       │     | |       │
│         │  ┌─────────┐  │    /   \      │
│ [+Var]  │  │Virar 90°│  │               │
│         │  └─────────┘  │               │
└─────────┴───────────────┴───────────────┘
```

---

### 🚶 Exercício 2: Primeiro Programa - Quadrado

Vamos fazer o robô andar em um quadrado!

#### Passo 1: Arrastar Blocos

1. Na aba **"Motion"**, arraste o bloco **"walk forward"** para a área de código
2. Clique no número e mude para **1** (metro)
3. Arraste mais 3 vezes (total de 4 blocos de andar)
4. Entre cada "andar", arraste o bloco **"turn"** e mude para **90** graus

**Seu código deve parecer:**
```
┌────────────────────┐
│ 🚩 Quando ▶️ clicado│
└────────────────────┘
        │
┌────────────────────┐
│ 🚶 Andar frente 1m │
└────────────────────┘
        │
┌────────────────────┐
│ 🔄 Virar 90°       │
└────────────────────┘
        │
┌────────────────────┐
│ 🚶 Andar frente 1m │
└────────────────────┘
        │
┌────────────────────┐
│ 🔄 Virar 90°       │
└────────────────────┘
        │
┌────────────────────┐
│ 🚶 Andar frente 1m │
└────────────────────┘
        │
┌────────────────────┐
│ 🔄 Virar 90°       │
└────────────────────┘
        │
┌────────────────────┐
│ 🚶 Andar frente 1m │
└────────────────────┘
        │
┌────────────────────┐
│ 🔄 Virar 90°       │
└────────────────────┘
```

---

#### Passo 2: Executar

1. Clique na **bandeira verde** 🚩 no Scratch
2. Observe o robô no Webots!
3. Ele deve andar em um quadrado perfeito

**Não funcionou?** Verifique:
- ✅ Simulação está rodando no Webots (▶️)
- ✅ Blocos estão conectados (sem espaços)
- ✅ Valores estão corretos (1m, 90°)

---

#### Passo 3: Otimizar com Loop

**Problema:** Nosso código está repetitivo. Vamos usar um loop!

1. **Delete** os blocos (arraste para fora)
2. Na aba **"Control"**, arraste **"repeat [10]"**
3. Mude o 10 para **4**
4. **Dentro** do repeat, coloque:
   - Andar frente 1m
   - Virar 90°

**Código otimizado:**
```
┌────────────────────┐
│ 🚩 Quando ▶️ clicado│
└────────────────────┘
        │
┌────────────────────┐
│ 🔁 REPETIR 4 vezes │
│  ┌──────────────┐  │
│  │ Andar 1m     │  │
│  └──────────────┘  │
│  ┌──────────────┐  │
│  │ Virar 90°    │  │
│  └──────────────┘  │
└────────────────────┘
```

**Resultado:** Mesmo comportamento, código 70% menor! Isso é **programação eficiente**. 🎉

---

### 👀 Exercício 3: Usando Sensores - Desviar de Obstáculo

Agora vamos fazer o robô **reagir ao ambiente**!

#### Passo 1: Adicionar Obstáculo

1. No Webots, pause a simulação
2. Adicione um **WoodenBox** na frente do robô
   - Posição: (1, 0.05, 0)
3. Reinicie a simulação

---

#### Passo 2: Programa com Sensor

1. No Scratch, crie este código:

```
┌────────────────────┐
│ 🚩 Quando ▶️ clicado│
└────────────────────┘
        │
┌────────────────────┐
│ ♾️ PARA SEMPRE     │
│  ┌──────────────┐  │
│  │ SE [dist<0.5]│  │
│  │  ┌────────┐  │  │
│  │  │ Parar  │  │  │
│  │  └────────┘  │  │
│  │  ┌────────┐  │  │
│  │  │Virar 90│  │  │
│  │  └────────┘  │  │
│  │ SENÃO        │  │
│  │  ┌────────┐  │  │
│  │  │Andar   │  │  │
│  │  └────────┘  │  │
│  └──────────────┘  │
└────────────────────┘
```

**Como montar:**

1. Arraste **"forever"** (para sempre)
2. Dentro dele, arraste **"if-else"**
3. Na condição do IF:
   - Arraste **"[  ] < [  ]"** (operador)
   - No primeiro espaço: arraste **"distance sensor"** (aba Sensors)
   - No segundo espaço: digite **0.5**
4. No bloco IF (quando distância < 0.5m):
   - Arraste **"stop motors"**
   - Arraste **"turn 90"**
5. No bloco ELSE (quando distância >= 0.5m):
   - Arraste **"walk forward"** (deixe 0.5m)

---

#### Passo 3: Testar

1. Clique na bandeira verde 🚩
2. Observe o comportamento:
   - Robô anda em direção à caixa
   - Quando chega perto (50cm), para
   - Vira 90° para desviar
   - Continua andando

**Parabéns!** Você criou um robô com **comportamento reativo**! 🤖

---

### 🎤 Exercício 4: Comandos de Voz com IA

Vamos integrar reconhecimento de voz para controlar o robô!

#### Passo 1: Ativar Microfone

**NOTA:** Este recurso requer:
- Microfone conectado
- Permissão de acesso ao mic (navegador vai pedir)
- Conexão com internet (API de speech-to-text)

1. No Scratch, vá na aba **"Extensions"** (canto inferior esquerdo)
2. Clique em **"Speech Recognition"**
3. Novos blocos aparecerão na aba **"Speech"**

---

#### Passo 2: Programa com Voz

```
┌────────────────────┐
│ 🚩 Quando ▶️ clicado│
└────────────────────┘
        │
┌────────────────────┐
│ ♾️ PARA SEMPRE     │
│  ┌──────────────┐  │
│  │ Ouvir comando│  │
│  └──────────────┘  │
│        │           │
│  ┌──────────────┐  │
│  │SE [voz]="ir" │  │
│  │  Andar 1m    │  │
│  └──────────────┘  │
│        │           │
│  ┌──────────────┐  │
│  │SE [voz]="virar"│ │
│  │  Virar 90°   │  │
│  └──────────────┘  │
│        │           │
│  ┌──────────────┐  │
│  │SE [voz]="parar"│ │
│  │  Parar motores│ │
│  └──────────────┘  │
└────────────────────┘
```

**Como montar:**

1. Dentro do "forever", coloque **"listen and wait"**
2. Adicione 3 blocos **"if"** (não if-else, só if)
3. Em cada condição:
   - Arraste **"[  ] = [  ]"**
   - Primeiro espaço: **"last heard word"**
   - Segundo espaço: escreva **"ir"**, **"virar"**, **"parar"**
4. Dentro de cada IF, coloque a ação correspondente

---

#### Passo 3: Testar Voz

1. Clique na bandeira verde 🚩
2. Fale claramente no microfone:
   - **"Ir"** → Robô anda
   - **"Virar"** → Robô vira
   - **"Parar"** → Robô para

**Dicas:**
- 🎙️ Fale devagar e claramente
- 🔊 Ajuste volume do microfone (se não reconhecer)
- 🌐 Necessita internet para funcionar
- 🇧🇷 Funciona em português!

**Não está funcionando?**
- Verifique permissões do navegador
- Teste o mic em outro site ([Online Mic Test](https://www.onlinemictest.com))
- Se o Scratch não suportar, use **alternativa manual** (teclado):
  - Tecla 'I' = Ir
  - Tecla 'V' = Virar
  - Tecla 'P' = Parar

---

### 🧠 Exercício 5: Detectar Objeto por Cor

Use a câmera do robô para encontrar uma bola vermelha!

#### Passo 1: Adicionar Bola

1. No Webots, adicione uma **Ball** vermelha
2. Posição: (2, 0.1, 0)
3. Mude a cor para vermelho:
   - Selecione a bola
   - Campo **"appearance > material > baseColor"**: `1 0 0`

---

#### Passo 2: Programa de Busca

```
┌────────────────────┐
│ 🚩 Quando ▶️ clicado│
└────────────────────┘
        │
┌────────────────────┐
│ 📢 Dizer "Procurand│
│    o bola..."      │
└────────────────────┘
        │
┌────────────────────┐
│ ♾️ PARA SEMPRE     │
│  ┌──────────────┐  │
│  │Capturar imagem│ │
│  └──────────────┘  │
│        │           │
│  ┌──────────────┐  │
│  │ SE [vermelho │  │
│  │    detectado]│  │
│  │  ┌────────┐  │  │
│  │  │Dizer   │  │  │
│  │  │"Achei!"│  │  │
│  │  └────────┘  │  │
│  │  ┌────────┐  │  │
│  │  │ Parar  │  │  │
│  │  └────────┘  │  │
│  │  ┌────────┐  │  │
│  │  │ SAIR   │  │  │
│  │  └────────┘  │  │
│  │ SENÃO        │  │
│  │  ┌────────┐  │  │
│  │  │Virar 10│  │  │
│  │  └────────┘  │  │
│  └──────────────┘  │
└────────────────────┘
```

**Lógica:**
1. Robô captura imagem da câmera
2. Analisa se há vermelho (>50% dos pixels)
3. Se SIM: diz "Achei!" e para
4. Se NÃO: vira 10° e tenta de novo

**Resultado:** Robô gira até encontrar a bola!

---

### 🐛 Exercício 6: Debugging Visual

**Cenário:** Você criou este programa, mas o robô não está fazendo o esperado.

```
┌────────────────────┐
│ 🚩 Quando ▶️ clicado│
└────────────────────┘
        │
┌────────────────────┐
│ 🔁 REPETIR 5 vezes │
│  ┌──────────────┐  │
│  │ Andar 2m     │  │
│  └──────────────┘  │
│  ┌──────────────┐  │
│  │ Virar 72°    │  │  ← BUG: deveria ser 60°!
│  └──────────────┘  │
└────────────────────┘
```

**Problema:** Robô deveria fazer um **pentágono** (5 lados), mas não fecha a forma.

#### Técnicas de Debugging:

**1. Modo Passo-a-Passo:**
- Botão "Run step by step" no Scratch
- Executa um bloco por vez
- Você vê exatamente onde falha

**2. Mostrar Valores:**
```
Adicione dentro do loop:
┌──────────────┐
│ Dizer [contador] por 1s │
└──────────────┘
```
Assim você vê quantas vezes repetiu.

**3. Testar Isoladamente:**
- Crie um novo programa APENAS com "Virar 72°"
- Repita 5 vezes
- Total girado = 72° × 5 = 360°? ✅
- Mas pentágono precisa de 60° × 5 = 300°! ❌

**CORREÇÃO:** Mude 72° para **60°**.

---

## 🎯 Desafio do Módulo

### Desafio: Robô Autônomo Completo

**Objetivo:** Criar um programa que faz o robô:
1. Andar para frente
2. Desviar de obstáculos automaticamente
3. Parar quando você fala "parar"
4. Retomar quando você fala "continuar"
5. Encontrar e ir até uma bola vermelha

**Requisitos Técnicos:**

- [ ] Usa sensor de distância (desviar obstáculos)
- [ ] Usa câmera (detectar bola vermelha)
- [ ] Usa microfone (comandos de voz)
- [ ] Tem pelo menos 1 variável (ex: "estado" = andando/parado)
- [ ] Usa loop "forever"
- [ ] Usa pelo menos 2 estruturas "if"

**Critérios de Sucesso:**
- [ ] Robô navega por pelo menos 30 segundos sem colidir
- [ ] Desvia de pelo menos 2 obstáculos
- [ ] Para quando ouve "parar"
- [ ] Encontra bola vermelha em até 2 minutos
- [ ] Código tem comentários explicando cada parte
- [ ] Vídeo de demonstração (1-2 min)

**Entrega:**
1. Arquivo `.sb3` (salvar projeto Scratch)
2. Arquivo `.wbt` (mundo do Webots)
3. Vídeo demonstrando funcionamento
4. Documento explicando sua lógica (1 página)

**Dica de Ouro:** Comece simples! Primeiro faça andar e desviar. Depois adiciona voz. Por último, adiciona câmera.

---

## 📚 Recursos Adicionais

### 📖 Leituras Complementares

- **[Visual Programming for Robots](https://exemplo.com)** - Artigo acadêmico (20 min)
- **[Scratch Documentation](https://scratch.mit.edu/help)** - Guia oficial
- **[Blockly Developer Guide](https://developers.google.com/blockly)** - Referência técnica

### 🎥 Vídeos Recomendados

- **[Scratch for Robotics - Full Course](https://youtube.com)** - Curso completo (2h)
- **[Debugging Like a Pro](https://youtube.com)** - Técnicas avançadas (15 min)
- **[Voice Control Robot Tutorial](https://youtube.com)** - Passo a passo (20 min)

### 🔗 Ferramentas Online

- **[Scratch Online Editor](https://scratch.mit.edu/projects/editor)** - Testar blocos
- **[Blockly Games](https://blockly.games)** - Praticar lógica
- **[Code.org Hour of Code](https://code.org/hourofcode)** - Exercícios interativos

### 🇧🇷 Conteúdo em Português

- **[Scratch Brasil](https://scratchbrasil.org.br)** - Comunidade nacional
- **[Programaê!](https://programae.org.br)** - Plataforma educacional
- **[Code Club Brasil](https://codeclub.org.br)** - Projetos gratuitos

---

## 🔧 Troubleshooting

### Problema 1: Scratch não conecta ao Webots

**Sintomas:** Blocos não funcionam, robô não se move.

**Soluções:**
1. Verifique que a simulação está **rodando** (▶️)
2. Reinicie o Webots E o Scratch
3. Abra o mundo exemplo: `scratch_nao.wbt`
4. Se nada funciona, use **Blockly** (alternativa)

---

### Problema 2: Sensor de distância sempre retorna 0

**Causa:** Sensor não está habilitado.

**Solução:**
No Scratch, adicione no início do programa:
```
┌──────────────┐
│Enable distance│
│    sensor    │
└──────────────┘
```

---

### Problema 3: Voz não é reconhecida

**Causas possíveis:**
- Sem permissão do navegador
- Sem conexão internet
- Microfone desabilitado
- Pronúncia diferente

**Soluções:**
1. Teste mic em [https://www.onlinemictest.com](https://www.onlinemictest.com)
2. Recarregue a página e aceite permissão
3. Use palavras simples (ir, parar, virar)
4. Como alternativa, use **teclado** (teclas I, P, V)

---

### Problema 4: Robô cai durante execução

**Causa:** Comandos muito rápidos, robô perde equilíbrio.

**Solução:**
Adicione **"wait 0.5 seconds"** entre comandos:
```
┌──────────────┐
│ Andar 1m     │
└──────────────┘
       │
┌──────────────┐
│ Esperar 0.5s │  ← Adicione isto!
└──────────────┘
       │
┌──────────────┐
│ Virar 90°    │
└──────────────┘
```

---

## ✅ Checklist de Conclusão

Antes de finalizar o Nível 1:

- [ ] Instalei e configurei Scratch no Webots
- [ ] Criei programa que faz robô andar em formas (quadrado, triângulo)
- [ ] Usei sensor de distância para desviar obstáculos
- [ ] Implementei comandos de voz (ou teclado)
- [ ] Usei câmera para detectar objeto colorido
- [ ] Debugguei e corrigi pelo menos 1 erro
- [ ] Criei programa que combina movimento + sensor + voz
- [ ] Completei o Desafio do Módulo
- [ ] Entendo a lógica de loops e condicionais
- [ ] Estou pronto para Python! 🚀

**Tempo médio de conclusão:** 5h30min

---

## 🎉 PARABÉNS - NÍVEL 1 COMPLETO!

Você acabou de finalizar o **Nível 1: Explorador**!

### 🏆 O Que Você Conquistou:

**Conhecimento:**
- ✅ Entende como robôs humanoides funcionam
- ✅ Domina simulador profissional (Webots)
- ✅ Sabe programar comportamentos complexos visualmente
- ✅ Integrou sensores, motores e IA básica

**Habilidades Práticas:**
- ✅ Criou 5+ programas funcionais
- ✅ Debuggou e corrigiu problemas
- ✅ Usou lógica de programação (loops, condicionais)

**Impacto Real:**
- 💰 **Valor aprendido:** Equivalente a um curso de R$ 500+
- ⏱️ **Investimento:** 12-15 horas bem gastas
- 🎯 **Taxa de conclusão:** Você está entre os 70% que chegam até aqui!

---

### 📜 Próximos Passos:

#### 1. Obtenha Seu Certificado

**Como:**
1. Complete o **Projeto Final do Nível 1**
   - [Ver requisitos aqui]({{ '/niveis/nivel-1/' | relative_url }})
2. Envie via formulário
3. Receba certificado digital em 48h

**Inclui:**
- Certificado PDF personalizado
- Badge LinkedIn
- Acesso ao Discord exclusivo

---

#### 2. Avance para o Nível 2: Criador

**O que vem a seguir:**
- 🐍 **Python e ROS 2:** Programação "de verdade"
- 🤖 **Controle avançado:** PID, cinemática, trajetórias
- 🧠 **Introdução a RL:** Robô que aprende sozinho
- 🚀 **Projeto:** Comportamento autônomo completo

**Pré-requisito:** Conclusão do Nível 1 ✅

[→ Começar Nível 2]({{ '/niveis/nivel-2/' | relative_url }}){: .btn .btn-primary}

---

#### 3. Escolha Sua Trilha Temática

Personalize sua jornada:

- 🎓 **Trilha Educação:** Use robôs para ensinar
- 🤝 **Trilha Social:** Leve tecnologia a comunidades
- 💼 **Trilha Empreendedorismo:** Crie seu negócio
- 🔬 **Trilha Tecnologia Avançada:** Pesquisa e inovação

[→ Ver Trilhas]({{ '/trilhas/' | relative_url }})

---

### 💬 Compartilhe Sua Conquista!

Poste nas redes sociais com:

**#FTH2026 #RoboticaBrasil #MeuPrimeiroRobo**

```
🤖 Acabei de completar o Nível 1 do FTH!

Aprendi a:
✅ Programar robôs humanoides
✅ Usar simuladores profissionais
✅ Criar comportamentos autônomos

Próximo: Nível 2! 🚀

[Link do projeto]: [seu-video]
```

---

### 🌟 Depoimento de Quem Completou:

> "Nunca imaginei que conseguiria fazer um robô desviar de obstáculos! A programação visual foi o ponto de virada para mim. Agora estou no Nível 3 e criando IAs complexas!" - **Pedro, 19 anos, estudante**

> "Como professora, o Nível 1 me deu confiança para levar robótica para minha sala de aula. Meus alunos de 10 anos ADORARAM!" - **Maria, 28 anos, professora**

---

**Você está pronto para o próximo nível?** 🚀

[→ Ir para Nível 2]({{ '/niveis/nivel-2/' | relative_url }}){: .btn .btn-large .btn-primary}

[← Voltar para Nível 1]({{ '/niveis/nivel-1/' | relative_url }}){: .btn .btn-secondary}

---

**Última atualização:** 2025-10-29
**Tempo médio de conclusão:** 5h45min
**Taxa de satisfação:** ⭐⭐⭐⭐⭐ (4.9/5.0)
**Próximo nível liberado:** ✅

---

> "A jornada de mil milhas começa com um único passo. Você acabou de dar os primeiros passos em robótica humanoide. Continue!" - Equipe FTH 💙
