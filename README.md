# Navegação Autônoma e Desvio de Obstáculos com o Robô Khepera IV

## 🤖 Sobre o Projeto
Este projeto consiste no desenvolvimento de um sistema de controle embarcado reativo para o robô móvel **Khepera IV** (Versão 3.0). O objetivo principal é fazer com que o robô se desloque de forma totalmente autônoma de uma extremidade a outra de uma sala de aula (terreno plano), detectando e desviando de obstáculos inicialmente desconhecidos (resmas de papel) dispostos ao longo do trajeto.

Trabalho desenvolvido como atividade prática para a disciplina de **Engenharia de Software** do Centro de Pesquisa e Desenvolvimento em Tecnologia Eletrônica e da Informação (**CETELI - UFAM**).

---

## 👥 Organização do Grupo e Papéis
O desenvolvimento do sistema foi dividido de forma modular e baseada em componentes, com as seguintes responsabilidades:
* **Ailesson Nando**: Gestão de Projeto, Documentação e Garantia de Qualidade (QA).
* **Alexandre Antonaccio**: Concepção de Desenvolvimento, Integração e Arquitetura do Sistema.
* **Alberth Viana**: Engenharia de Software — Módulo de Percepção e Filtros de Sensores.
* **Fernanda Costa**: Engenharia de Software — Módulo de Atuação e Controle de Motores/Odometria.
* **Matheus Reges**: Engenharia de Software — Lógica de Decisão e Máquina de Estados (FSM).

---

## 🏗️ Arquitetura do Software
O sistema utiliza uma arquitetura baseada em componentes e orientada a eventos para garantir o desacoplamento dos módulos físicos e lógicos. O fluxo segue o ciclo clássico da robótica: **Percepção ➔ Processamento/Decisão ➔ Atuação**.


## khepera4-navegacao-autonoma/
├── Makefile                  
├── README.md                 
├── include/                  
│   ├── atuacao.h             
│   ├── decisao.h             
│   └── percepcao.h           
└── src/                      
├── atuacao.c
├── decisao.c
├── main.c                
└── percepcao.c

---

## 🛠️ Requisitos do Ambiente

### Hardware
* 01 Robô Móvel Autônomo Khepera IV (K-Team).
* Computador de desenvolvimento com interface de rede ativa (Wi-Fi) para comunicação SSH/SCP.
* Obstáculos físicos (Resmas de Papel A4).

### Software & Ferramentas
* **Simulador:** Webots (com o modelo nativo do Khepera IV instalado).
* **Compilador:** GCC com suporte a Makefiles e a biblioteca base `khepera4toolbox`.
* **Editor:** Visual Studio Code ou qualquer IDE C/C++.

---

## Como Compilar e Executar

### 1. No Ambiente de Simulação (Webots)
Para testar a lógica localmente através de um controlador externo:
1. Abra o Webots e carregue o mundo de testes em `Open Sample World` ➔ `robots` ➔ `k-team` ➔ `khepera4.wbt`.
2. Altere o campo `controller` do robô na Scene Tree para **`extern`**.
3. No terminal do seu computador, navegue até a raiz deste projeto e compile o código:
   ```bash
   make