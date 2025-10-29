# Product Requirements Document (PRD)
# Programa FTH - Formação Tecnológica em Humanoides

**Versão:** 1.0
**Data:** 2025-10-29
**Product Manager:** John
**Status:** Draft → Aprovação

---

## 1. Executive Summary

### Visão
Democratizar o acesso ao conhecimento em **robótica humanoide e inteligência artificial**, formando pessoas de diferentes níveis socioeconômicos para dominar o uso, programação e desenvolvimento de robôs humanoides, tanto em ambiente virtual quanto real.

### Missão
Criar um **ecossistema educacional sustentável** que:
- Forma talentos em robótica e IA
- Estimula empreendedorismo tecnológico
- Integra robótica à educação em todos os níveis
- Gera impacto social e econômico mensurável

### Proposta de Valor
**"Do Zero ao Robô Autônomo"** - Um programa educacional completo, progressivo e acessível que permite qualquer pessoa, independente do background, aprender a criar e programar robôs humanoides com IA.

---

## 2. Objetivos Estratégicos

### Objetivos de Negócio
1. **Alcance:** Formar 1.000+ pessoas no primeiro ano
2. **Impacto Social:** 30% dos alunos de comunidades carentes
3. **Empregabilidade:** 60% dos formados aplicando conhecimento profissionalmente
4. **Sustentabilidade:** Modelo autossustentável após 18 meses
5. **Ecossistema:** Parcerias com 20+ escolas/empresas no primeiro ano

### Objetivos Educacionais
1. Taxa de conclusão > 70% (nível 1)
2. Taxa de progressão entre níveis > 50%
3. Satisfação do aluno (NPS) > 8.0
4. Aplicação prática dos conhecimentos > 80%

### Métricas de Sucesso (KPIs)

| Métrica | Meta Ano 1 | Meta Ano 2 |
|---------|-----------|-----------|
| Alunos matriculados | 1.500 | 5.000 |
| Taxa de conclusão Nível 1 | 70% | 75% |
| Projetos finais criados | 300 | 1.500 |
| Startups/negócios gerados | 10 | 50 |
| Parcerias institucionais | 20 | 100 |
| NPS (Net Promoter Score) | 8.0 | 8.5 |

---

## 3. Análise de Público e Personas

### Públicos-Alvo

#### Persona 1: **Maria - A Curiosa**
- **Perfil:** 28 anos, professora do ensino fundamental
- **Contexto:** Quer modernizar suas aulas, nunca programou
- **Objetivo:** Entender como robôs funcionam para ensinar seus alunos
- **Caminho:** Nível 1 (Explorador) → Trilha Educação
- **Barreira:** Medo de tecnologia, falta de tempo
- **Motivação:** Engajar alunos com tecnologia prática

#### Persona 2: **Pedro - O Maker**
- **Perfil:** 19 anos, estudante de engenharia
- **Contexto:** Faz Arduino e Raspberry Pi, quer evoluir para IA
- **Objetivo:** Criar robôs autônomos e participar de competições
- **Caminho:** Nível 2 (Criador) → Nível 3 (Desenvolvedor)
- **Barreira:** Custo de hardware, falta de mentoria
- **Motivação:** Domínio técnico e portfólio profissional

#### Persona 3: **Carlos - O Empreendedor**
- **Perfil:** 35 anos, dono de pequena empresa de eventos
- **Contexto:** Vê potencial em robôs para serviços e negócios
- **Objetivo:** Criar negócio de robôs recepcionistas/guias
- **Caminho:** Nível 1 → Nível 4 (Profissional) → Trilha Empreendedora
- **Barreira:** Investimento inicial, conhecimento técnico
- **Motivação:** Oportunidade de negócio inovador

#### Persona 4: **Ana - A Engenheira**
- **Perfil:** 42 anos, engenheira mecatrônica
- **Contexto:** Quer migrar para robótica com IA moderna
- **Objetivo:** Desenvolver sistemas autônomos avançados
- **Caminho:** Nível 3 (Desenvolvedor) → Trilha Tecnologia Avançada
- **Barreira:** Atualização em RL e LBMs
- **Motivação:** Relevância profissional e inovação

#### Persona 5: **João - O Jovem da Periferia**
- **Perfil:** 16 anos, estudante de escola pública
- **Contexto:** Tem acesso limitado a tecnologia, é curioso
- **Objetivo:** Descobrir uma carreira em tecnologia
- **Caminho:** Nível 1 (Explorador) → Clube de Robótica → Trilha Social
- **Barreira:** Acesso a computador/internet, falta de recursos
- **Motivação:** Mudança de vida, oportunidade profissional

---

## 4. Estrutura do Produto - Programa Educacional

### Arquitetura do Curso

```
FTH - Formação Tecnológica em Humanoides
│
├── NÍVEL 1: Explorador (12-15h)
│   ├── Módulo 1: O que são Robôs Humanoides?
│   ├── Módulo 2: Primeiros Passos na Simulação
│   └── Projeto: Fazer o robô andar e responder voz
│
├── NÍVEL 2: Criador (35-45h)
│   ├── Módulo 3: Programação com Python e ROS 2
│   ├── Módulo 4: Sensores e Controle
│   ├── Módulo 5: Introdução a Aprendizado por Reforço
│   └── Projeto: Criar comportamento autônomo
│
├── NÍVEL 3: Desenvolvedor (50-60h)
│   ├── Módulo 6: Algoritmos de IA (PPO, SAC)
│   ├── Módulo 7: Visão Computacional
│   ├── Módulo 8: Large Behavior Models (LBMs)
│   └── Projeto: Robô que aprende novas tarefas
│
├── NÍVEL 4: Profissional (30-40h)
│   ├── Módulo 9: Modelos de Negócio
│   ├── Módulo 10: Sim-to-Real Transfer
│   ├── Módulo 11: Protótipos e Aplicações
│   └── Projeto: Startup ou Aplicação Real
│
└── TRILHAS TEMÁTICAS (transversais)
    ├── Educação
    ├── Social
    ├── Empreendedorismo
    └── Tecnologia Avançada
```

---

## 5. Requisitos Funcionais Detalhados

### RF-001: Plataforma Web Educacional
**Prioridade:** P0 (Crítico)

**Descrição:** Site estático GitHub Pages com Jekyll para hospedar todo o conteúdo do curso.

**Requisitos:**
- Homepage com apresentação do programa
- Navegação clara entre níveis e módulos
- Páginas individuais para cada módulo com conteúdo estruturado
- Sistema de trilhas temáticas
- Página de recursos e ferramentas
- Responsivo (mobile, tablet, desktop)
- Acessibilidade (WCAG 2.1 AA)

**Critérios de Aceite:**
- [x] Usuário consegue navegar entre todos os níveis
- [x] Conteúdo carrega em < 3 segundos
- [x] Funciona em Chrome, Firefox, Safari, Edge
- [x] Mobile-friendly (responsive)

---

### RF-002: Conteúdo Didático Estruturado
**Prioridade:** P0 (Crítico)

**Descrição:** Conteúdo pedagógico completo dos 4 níveis e trilhas.

**Estrutura de Módulo:**
```markdown
# Módulo X: Título

## Objetivos de Aprendizado
- [ ] Objetivo 1
- [ ] Objetivo 2

## Conteúdo Teórico
- Explicação conceitual
- Exemplos práticos
- Diagramas/ilustrações

## Prática Hands-On
- Passo a passo tutorial
- Código de exemplo
- Desafios

## Projeto/Exercício
- Descrição do desafio
- Critérios de sucesso
- Recursos adicionais

## Recursos
- Links externos
- Ferramentas necessárias
- Leitura complementar
```

---

### RF-003: Sistema de Progressão
**Prioridade:** P1 (Alta)

**Descrição:** Indicadores visuais de progressão e conclusão de módulos.

**Requisitos:**
- Checkboxes para marcar módulos concluídos
- Badges de conquista por nível
- Indicador de progresso visual (barra)
- Certificado digital ao final de cada nível

---

### RF-004: Recursos e Ferramentas
**Prioridade:** P0 (Crítico)

**Descrição:** Página centralizada com todas as ferramentas necessárias.

**Ferramentas a Documentar:**
- Simuladores: Isaac Sim, Gazebo, Webots, MuJoCo
- Linguagens: Python, ROS 2
- Bibliotecas: TensorFlow, PyTorch, OpenCV
- Plataformas: Google Colab, RoboFlow
- Hardware: Noetix Bumi, Unitree R1, Raspberry Pi

**Formato:**
- Nome da ferramenta
- Descrição e aplicação
- Link para download/instalação
- Tutorial de setup
- Quando usar no curso

---

### RF-005: Projetos Práticos
**Prioridade:** P0 (Crítico)

**Descrição:** Cada nível termina com um projeto prático tangível.

**Projetos:**
1. **Nível 1:** Fazer robô andar e responder comandos de voz
2. **Nível 2:** Criar comportamento autônomo (desviar obstáculos)
3. **Nível 3:** Robô aprende nova tarefa (pegar objeto)
4. **Nível 4:** Protótipo aplicado (startup/escola/empresa)

**Requisitos por Projeto:**
- Descrição clara do objetivo
- Instruções passo a passo
- Código template (GitHub)
- Critérios de avaliação
- Galeria de projetos da comunidade

---

### RF-006: Trilhas Temáticas
**Prioridade:** P1 (Alta)

**Descrição:** Caminhos personalizados de aplicação do conhecimento.

**Trilhas:**

#### 🎓 Trilha Educação
- **Objetivo:** Usar robôs como ferramenta didática
- **Público:** Professores, pedagogos, escolas
- **Projeto Final:** Robô tutor para ensinar matemática/ciências
- **Conteúdo Extra:** Metodologias ativas, STEAM education

#### 🤝 Trilha Social
- **Objetivo:** Levar conhecimento a comunidades
- **Público:** ONGs, líderes comunitários, jovens
- **Projeto Final:** Oficina de robótica para jovens de baixa renda
- **Conteúdo Extra:** Acessibilidade, inclusão digital

#### 💼 Trilha Empreendedorismo
- **Objetivo:** Criar negócios com robôs
- **Público:** Empreendedores, empresários
- **Projeto Final:** Startup ou serviço com robôs
- **Conteúdo Extra:** Business model canvas, pitch, MVP

#### 🔬 Trilha Tecnologia Avançada
- **Objetivo:** Pesquisa e desenvolvimento em IA/robótica
- **Público:** Engenheiros, pesquisadores, devs
- **Projeto Final:** Sistema autônomo complexo
- **Conteúdo Extra:** Papers, benchmarks, state-of-the-art

---

## 6. Requisitos Não-Funcionais

### RNF-001: Performance
- Carregamento de página < 3 segundos
- Imagens otimizadas (< 500KB cada)
- Lazy loading de vídeos e recursos pesados

### RNF-002: Acessibilidade
- WCAG 2.1 nível AA
- Navegação por teclado
- Screen reader friendly
- Contraste adequado (mínimo 4.5:1)

### RNF-003: SEO
- Meta tags adequadas
- Sitemap XML
- URLs semânticas
- Schema.org markup para conteúdo educacional

### RNF-004: Analytics
- Google Analytics para tracking de uso
- Eventos personalizados (conclusão de módulos)
- Heatmaps de navegação

### RNF-005: Manutenibilidade
- Código bem documentado
- Markdown para todo conteúdo
- Estrutura modular e escalável
- Versionamento Git com commits semânticos

---

## 7. Tecnologias e Stack Técnico

### Frontend
- **Jekyll** - Gerador de site estático
- **Markdown** - Conteúdo dos módulos
- **HTML5/CSS3** - Layout responsivo
- **JavaScript vanilla** - Interatividade leve

### Hospedagem
- **GitHub Pages** - Hosting gratuito
- **GitHub Actions** - CI/CD automático
- **Git** - Controle de versão

### Tema/Design
- Baseado no estilo FEP (https://inematds.github.io/FEP/)
- Sistema de cards para módulos
- Emojis como ícones visuais
- Paleta de cores moderna e acessível

### Ferramentas de Desenvolvimento
- **VS Code** - Editor
- **Git Bash / Terminal** - CLI
- **Markdown Preview** - Visualização local

---

## 8. Roadmap de Implementação

### Fase 1: Fundação (Semanas 1-2)
**Objetivo:** Criar estrutura básica e conteúdo core

**Entregas:**
- ✅ Repositório GitHub FTH criado
- ✅ GitHub Pages configurado
- ✅ Homepage com hero section
- ✅ Estrutura de navegação
- ✅ Conteúdo Nível 1 completo (3 módulos)
- ✅ Página de recursos

**Responsável:** Equipe Core
**Data:** 2025-11-12

---

### Fase 2: Expansão (Semanas 3-4)
**Objetivo:** Completar níveis 2 e 3

**Entregas:**
- ✅ Conteúdo Nível 2 completo (3 módulos)
- ✅ Conteúdo Nível 3 completo (3 módulos)
- ✅ Projetos práticos com código template
- ✅ Sistema de progressão visual
- ✅ Página "Sobre" e "FAQ"

**Responsável:** Equipe Core + Instrutores
**Data:** 2025-11-26

---

### Fase 3: Profissionalização (Semanas 5-6)
**Objetivo:** Nível 4 e trilhas temáticas

**Entregas:**
- ✅ Conteúdo Nível 4 completo (3 módulos)
- ✅ 4 Trilhas temáticas estruturadas
- ✅ Galeria de projetos da comunidade
- ✅ Certificados digitais
- ✅ Página de parcerias

**Responsável:** Equipe Completa
**Data:** 2025-12-10

---

### Fase 4: Lançamento e Crescimento (Semanas 7-8)
**Objetivo:** Go-live e primeiras turmas

**Entregas:**
- ✅ SEO otimizado
- ✅ Analytics configurado
- ✅ Marketing e comunicação
- ✅ Primeira turma piloto (50 alunos)
- ✅ Feedback loop implementado

**Responsável:** Equipe + Marketing
**Data:** 2025-12-24

---

### Fase 5: Melhoria Contínua (Ongoing)
**Objetivo:** Iterar baseado em feedback

**Atividades:**
- Análise de métricas semanalmente
- Atualização de conteúdo
- Novos módulos e recursos
- Parcerias e expansão

---

## 9. Modelo de Negócio

### Estratégia de Receita

#### Modelo Freemium
- **Gratuito:** Níveis 1 e 2 completos
- **Pago:** Níveis 3 e 4 + Certificação + Mentoria

#### Streams de Receita
1. **Certificações:** R$ 97 por nível
2. **Mentoria Premium:** R$ 297/mês (1h/semana)
3. **Parcerias Corporativas:** R$ 5.000+ (licença institucional)
4. **Hardware/Kits:** Venda de kits Noetix Bumi educacionais
5. **Consultorias:** Implementação em escolas/empresas

#### Projeção Financeira (Ano 1)

| Fonte | Meta | Receita Estimada |
|-------|------|------------------|
| Certificações (500 alunos) | R$ 97 x 500 | R$ 48.500 |
| Mentoria Premium (50 alunos) | R$ 297 x 50 x 12 | R$ 178.200 |
| Parcerias (10 instituições) | R$ 5.000 x 10 | R$ 50.000 |
| Hardware (100 kits) | R$ 1.500 x 100 | R$ 150.000 |
| **TOTAL ANO 1** | | **R$ 426.700** |

### Custos Operacionais (Ano 1)
- Infraestrutura: R$ 0 (GitHub Pages gratuito)
- Equipe (3 instrutores part-time): R$ 180.000
- Marketing: R$ 50.000
- Hardware protótipos: R$ 30.000
- Operacional: R$ 20.000
- **TOTAL:** R$ 280.000

**Lucro projetado Ano 1:** R$ 146.700

---

## 10. Riscos e Mitigações

### Risco 1: Baixa taxa de conclusão
**Probabilidade:** Alta | **Impacto:** Alto

**Mitigação:**
- Gamificação e badges de progresso
- Comunidade Discord ativa
- Mentorias em grupo semanais
- Checkpoints de motivação

---

### Risco 2: Complexidade técnica afasta iniciantes
**Probabilidade:** Média | **Impacto:** Alto

**Mitigação:**
- Nível 1 extremamente simplificado
- Vídeos tutoriais passo a passo
- Suporte via FAQ extensivo
- "Office hours" para dúvidas

---

### Risco 3: Falta de recursos/hardware
**Probabilidade:** Alta | **Impacto:** Médio

**Mitigação:**
- Foco total em simulação (primeiros níveis)
- Parcerias para acesso a labs
- Programa de bolsas para hardware
- Eventos presenciais em labs parceiros

---

### Risco 4: Concorrência de cursos gratuitos (Coursera, YouTube)
**Probabilidade:** Alta | **Impacto:** Médio

**Mitigação:**
- Diferencial: foco em humanoides (nicho)
- Comunidade brasileira forte
- Projetos práticos tangíveis
- Certificação reconhecida
- Trilha de empreendedorismo única

---

### Risco 5: Conteúdo desatualizado rapidamente (IA evolui rápido)
**Probabilidade:** Alta | **Impacto:** Médio

**Mitigação:**
- Revisão trimestral de conteúdo
- Módulos "News & Updates"
- Comunidade contribui com atualizações
- Parcerias com empresas (Noetix, Unitree)

---

## 11. Métricas de Sucesso e KPIs

### Métricas de Engajamento
- **DAU (Daily Active Users):** > 100 em 3 meses
- **Tempo médio no site:** > 15 minutos
- **Taxa de retorno:** > 40% (usuários voltam)
- **Páginas por sessão:** > 4

### Métricas Educacionais
- **Taxa de conclusão Nível 1:** > 70%
- **Taxa de progressão (N1→N2):** > 50%
- **Projetos submetidos:** > 300 no ano 1
- **Satisfação (NPS):** > 8.0

### Métricas de Negócio
- **Conversão Free→Paid:** > 10%
- **CAC (Customer Acquisition Cost):** < R$ 100
- **LTV (Lifetime Value):** > R$ 500
- **Churn rate:** < 15% mensal (mentorias)

### Métricas de Impacto
- **Alunos de baixa renda:** > 30% dos inscritos
- **Mulheres:** > 25% dos inscritos
- **Empregabilidade:** > 60% aplicando conhecimento
- **Startups criadas:** > 10 no ano 1

---

## 12. Plano de Lançamento

### Pré-Lançamento (4 semanas antes)

**Semana -4: Soft Launch**
- Grupo fechado de 20 beta testers
- Coleta de feedback inicial
- Ajustes de conteúdo e UX

**Semana -3: Marketing Prep**
- Criar landing page de waitlist
- Anúncios em redes sociais
- Artigos sobre democratização da robótica
- Parcerias com influencers tech

**Semana -2: Press & Partnerships**
- Press release para mídia tech
- Anúncio de parcerias com escolas
- Webinar gratuito "Futuro da Robótica"

**Semana -1: Final Polish**
- Últimos ajustes baseados em beta
- Preparação de email marketing
- FAQ atualizado
- Suporte preparado

---

### Lançamento (Semana 0)

**Dia 1: Go Live**
- Abertura oficial do site
- Email para waitlist (500+ pessoas)
- Posts em redes sociais
- Evento online de lançamento

**Semana 1: Ativação**
- Acompanhamento diário de métricas
- Resposta rápida a bugs/issues
- Engajamento em comunidade
- Primeira turma de mentoria

---

### Pós-Lançamento (Primeiros 3 meses)

**Mês 1: Estabilização**
- Daily stand-ups para resolver problemas
- Coleta de feedback contínuo
- Ajustes de conteúdo
- Início das mentorias

**Mês 2: Crescimento**
- Segunda onda de marketing
- Casos de sucesso de alunos
- Parcerias adicionais
- Primeiro evento presencial

**Mês 3: Escala**
- Automação de processos
- Expansão de equipe (se necessário)
- Lançamento de nível 3 (se não incluído)
- Preparação para trilhas temáticas

---

## 13. Dependências e Premissas

### Dependências Técnicas
- GitHub Pages funcionando sem downtime
- Simuladores gratuitos continuarem disponíveis
- Bibliotecas Python/ROS 2 estáveis

### Dependências de Negócio
- Parcerias com pelo menos 2 escolas
- Acesso a hardware para demos
- Equipe de 2-3 instrutores dedicados

### Premissas
- Público tem acesso básico à internet
- Computadores modestos (4GB RAM) são suficientes para simulação inicial
- Comunidade brasileira de robótica engajada
- Demanda por upskilling em IA/robótica continua alta

---

## 14. Próximos Passos (Action Items)

### Imediatos (Próximas 48h)
- [ ] Criar repositório GitHub "FTH"
- [ ] Configurar GitHub Pages
- [ ] Implementar estrutura básica Jekyll
- [ ] Criar homepage (hero + navegação)
- [ ] Escrever conteúdo Módulo 1

### Curto Prazo (Próximas 2 semanas)
- [ ] Completar conteúdo Nível 1
- [ ] Criar página de recursos
- [ ] Desenvolver primeiro projeto prático
- [ ] Setup analytics
- [ ] Documentar ferramentas (Isaac Sim, Webots)

### Médio Prazo (Próximo mês)
- [ ] Completar Níveis 2 e 3
- [ ] Formar equipe de instrutores
- [ ] Criar comunidade Discord
- [ ] Desenvolver sistema de certificados
- [ ] Preparar beta test com 20 pessoas

---

## 15. Aprovação e Sign-off

| Stakeholder | Role | Status | Data |
|-------------|------|--------|------|
| Você (Founder) | Product Owner | ⏳ Aguardando | - |
| John (PM) | Product Manager | ✅ Aprovado | 2025-10-29 |
| Equipe Dev | Implementação | ⏳ Aguardando PRD | - |
| Instrutores | Conteúdo | ⏳ Aguardando PRD | - |

---

## 16. Anexos

### A. Referências
- Site FEP: https://inematds.github.io/FEP/
- Documento Base: Doc/Sugestao-Plano.txt
- NVIDIA Isaac Sim: https://developer.nvidia.com/isaac-sim
- ROS 2: https://docs.ros.org/

### B. Glossário
- **RL:** Reinforcement Learning (Aprendizado por Reforço)
- **LBM:** Large Behavior Models (Modelos Comportamentais Grandes)
- **ROS:** Robot Operating System
- **Sim-to-Real:** Transferência de simulação para robô real
- **PPO:** Proximal Policy Optimization
- **SAC:** Soft Actor-Critic

---

**Documento criado por:** John - Product Manager
**Para:** Programa FTH - Formação Tecnológica em Humanoides
**Versão:** 1.0 - Draft Inicial
**Última atualização:** 2025-10-29
