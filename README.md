# Navegação Autônoma e Desvio de Obstáculos com o Robô Khepera IV

## 🤖 Sobre o Projeto

Este projeto consiste no desenvolvimento de um sistema de controle embarcado reativo para o robô móvel **Khepera IV** — versão 3.0.

O objetivo principal é fazer com que o robô se desloque de forma totalmente autônoma de uma extremidade a outra de uma sala de aula, em terreno plano, detectando e desviando de obstáculos inicialmente desconhecidos, como resmas de papel, dispostos ao longo do trajeto.

Trabalho desenvolvido como atividade prática para a disciplina de **Engenharia de Software** do Centro de Pesquisa e Desenvolvimento em Tecnologia Eletrônica e da Informação (**CETELI - UFAM**).

---

## 👥 Organização do Grupo e Papéis

O desenvolvimento do sistema foi dividido de forma modular e baseada em componentes, com as seguintes responsabilidades:

* **Fernanda Costa**: Gestão de Projeto, Documentação e Garantia de Qualidade (QA).
* **Alexandre Antonaccio**: Concepção de Desenvolvimento, Integração e Arquitetura do Sistema.
* **Alberth Viana**: Engenharia de Software — Módulo de Percepção e Filtros de Sensores.
* **Fernanda Costa**: Engenharia de Software — Módulo de Atuação e Controle de Motores/Odometria.
* **Matheus Reges**: Engenharia de Software — Lógica de Decisão e Máquina de Estados (FSM).

---

## 🏗️ Arquitetura do Software

O sistema utiliza uma arquitetura baseada em componentes e orientada a eventos para garantir o desacoplamento entre os módulos físicos e lógicos.

O fluxo principal segue o ciclo clássico da robótica:

```text
Percepção → Processamento/Decisão → Atuação
```

### Componentes principais

* **Módulo de Percepção**: responsável pela leitura dos sensores e tratamento dos dados capturados.
* **Módulo de Decisão**: responsável pela lógica de controle, interpretação do ambiente e tomada de decisão.
* **Módulo de Atuação**: responsável pelo acionamento dos motores e controle do movimento do robô.

---

## 📁 Estrutura do Projeto

```text
khepera4-navegacao-autonoma/
├── Makefile
├── README.md
├── .gitignore
├── include/
│   ├── actuation.h
│   ├── decision.h
│   └── perception.h
└── src/
    ├── actuation.c
    ├── decision.c
    ├── main.c
    └── perception.c
```

---

## 🛠️ Requisitos do Ambiente

### Hardware

* 01 Robô Móvel Autônomo Khepera IV (K-Team).
* Computador de desenvolvimento com interface de rede ativa, preferencialmente Wi-Fi, para comunicação via SSH/SCP.
* Obstáculos físicos, como resmas de papel A4.

### Software e Ferramentas

* **Ambiente de compilação**: Docker instalado para cross-compilação ARM usando a imagem Ubuntu 20.04.
* **Toolchain de compilação**: `gcc-arm-linux-gnueabihf` e `make`.
* **Editor**: Visual Studio Code ou qualquer IDE com suporte a C/C++.
* **Acesso remoto**: SSH/SCP para envio e execução do binário no robô.

---

## 🚀 Como Compilar e Executar

### 1. Compilação para o Robô Físico

Como o robô possui uma arquitetura ARM embarcada, a compilação deve ser feita de forma isolada via Docker para gerar um binário compatível.

Na raiz do projeto, execute:

```bash
docker run --rm -v "$(pwd)":/workspace -w /workspace ubuntu:20.04 sh -c "apt-get update && apt-get install -y gcc-arm-linux-gnueabihf make && make clean && make"
```

Esse comando irá:

* Criar um container temporário com Ubuntu 20.04.
* Instalar as ferramentas necessárias para compilação ARM.
* Limpar compilações anteriores.
* Gerar o binário compatível com o robô.

---

### 2. Transferência e Execução no Robô

Após gerar o binário `navegacao_fisi_khepera`, envie o arquivo para a placa do Khepera e execute os comandos via terminal SSH.

#### Enviar o executável para o robô

Substitua o IP abaixo pelo endereço atual do robô:

```bash
scp navegacao_fisi_khepera root@192.168.15.135:/home/root/
```

#### Acessar o robô via SSH

```bash
ssh root@192.168.15.135
```

#### Executar a aplicação no robô

No terminal do robô, finalize processos antigos, conceda permissão de execução ao binário e execute a aplicação:

```bash
sudo killall -9 navegacao_fisi_khepera
chmod +x /home/root/navegacao_fisi_khepera
sudo /home/root/navegacao_fisi_khepera
```

---

### 3. Execução no Ambiente de Simulação

Para testar a lógica localmente através de um controlador externo no Webots, utilize a compilação nativa para o computador.

#### Passos

1. Abra o Webots.
2. Carregue o mundo de testes em:

```text
Open Sample World → robots → k-team → khepera4.wbt
```

3. Na **Scene Tree**, altere o campo `controller` do robô para:

```text
extern
```

4. No terminal do computador, execute:

```bash
make clean && make
```

> **Observação:** para o modo de simulação, certifique-se de comentar ou remover a flag `-DROBOT_FISICO` no `Makefile`.

---

## 📌 Observações Importantes

* O endereço IP do robô pode mudar de acordo com a rede utilizada.
* Antes de executar novamente o programa no robô, finalize processos antigos para evitar conflito de execução.
* O modo físico e o modo de simulação podem exigir configurações diferentes no `Makefile`.
* A comunicação com o robô depende da conexão correta entre o computador e a rede Wi-Fi utilizada pelo Khepera IV.

---

## 📄 Licença

Este projeto foi desenvolvido exclusivamente para fins acadêmicos, como parte das atividades práticas da disciplina de **Engenharia de Software** do **CETELI - UFAM**.
