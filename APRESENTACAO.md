# Roteiro para a apresentação

## 1. Antes de sair de casa

- [ ] Robô Khepera IV carregado.
- [ ] Notebook com o projeto (`khepera4-autonomous-navigation`) e Docker instalado.
- [ ] Confirma que `git status` está limpo / commitado (rodar `git log --oneline -5` pra saber o que está valendo).
- [ ] Resma de papel (ou objeto equivalente) pra servir de obstáculo.
- [ ] Fita crepe ou similar pra marcar no chão o ponto de partida e o ponto de chegada (~1,5m à frente, ver seção 4).
- [ ] Cabo de rede / confirmar que o robô e o notebook vão conseguir se enxergar na rede do local da apresentação (IP do robô: `192.168.15.135` — **se a rede do local for diferente, isso pode mudar**, testar com antecedência se possível).

## 2. Build (no notebook)

Na raiz do projeto:

```bash
docker run --rm -v "$(pwd)":/workspace -w /workspace ubuntu:20.04 sh -c \
  "apt-get update && apt-get install -y gcc-arm-linux-gnueabihf make && make clean && make"
```

Confirma no final que apareceu a linha do `arm-linux-gnueabihf-gcc ... -o navegacao_fisi_khepera ...` sem nenhum erro em vermelho.

## 3. Deploy (do notebook pro robô)

```bash
scp navegacao_fisi_khepera root@192.168.15.135:/home/root/
```

## 4. Preparar o ambiente físico

O alvo está fixo no código em `main.c` como **1500mm (1,5m) à frente, quase reto** (`alvo_x=1500, alvo_y=1`). Então:

- Marca o ponto de partida no chão (o robô sempre parte de lá com a frente apontada pra onde vai andar).
- Marca um ponto ~1,5m à frente em linha reta — é onde ele deve parar sozinho.
- Coloca a resma de papel **no meio do caminho**, de forma que bloqueie a rota direta.
- Deixa uns 50cm livres nas laterais pra não confundir os sensores com outra coisa.

## 5. Rodar a demonstração

Em um terminal, conecta no robô:

```bash
ssh root@192.168.15.135
cd /home/root
chmod +x ./navegacao_fisi_khepera
sudo ./navegacao_fisi_khepera
```

**Confirma que o prompt virou `root@khepera4_1400:~#` antes de rodar o `sudo`** — se ainda estiver `lse@...`, o SSH não conectou.

Pra parar a qualquer momento: `Ctrl+C` (o código trata o sinal e força o robô a zerar a velocidade antes de encerrar).

## 6. O que explicação pro professor enquanto ele anda

O robô segue uma máquina de estados reativa (`src/decision.c`). No terminal, a linha `[STATUS]` mostra o estado interno em tempo real (campo `fase`):

| fase | nome | o que está fazendo |
|---|---|---|
| 0 | conduzindo | indo em linha reta na direção do alvo (controlador proporcional de rumo) |
| 1 | desviando | girando parado no próprio eixo até a frente limpar do obstáculo |
| 2 | escape | andando reto uma distância curta de segurança, só pra se afastar do obstáculo |
| 3 | retomando trajetória | curvando de volta pra cima da reta original entre o ponto de partida e o alvo |

Sequência esperada numa passagem com obstáculo: `0 → 1 → 2 → 3 → 0` (parado ao chegar perto do alvo).

Pontos pra destacar:
- **Percepção**: sensores infravermelhos de proximidade (`src/perception.c`) detectam o obstáculo e indicam de que lado ele está mais forte.
- **Decisão**: a máquina de estados decide se vai reto, gira pra desviar, ou está retomando a trajetória original (`src/decision.c`).
- **Atuação**: velocidades de roda calculadas são enviadas ao driver físico do Khepera IV (`src/actuation.c`), com odometria por encoder pra saber a posição (x, y, ângulo) o tempo todo.

## 7. Se algo der errado na hora

- **`chmod: cannot access`** → você está no terminal local, não no robô. Roda `ssh root@192.168.15.135` de novo.
- **Robô não desvia / passa reto pelo obstáculo** → os sensores podem precisar de recalibração pro ambiente novo (iluminação/piso diferentes mudam a leitura). Threshold fica em `src/perception.c:26`.
- **Robô gira mas nunca volta pra reta / fica manco** → já foi corrigido nos testes anteriores (bug de unidades em `src/decision.c`), não deve mais acontecer — mas se acontecer, é o primeiro lugar pra olhar.
- **Perda de conexão SSH no meio da demo** → o robô continua rodando sozinho (é um processo já iniciado); só reconecta com `ssh root@192.168.15.135` se precisar ver o log de novo, ou aperta o botão físico de emergência do robô se precisar parar sem terminal.

## 8. Se o professor pedir pra mudar algo na hora

Não recomendo editar código ao vivo sob pressão, mas se pedir pra "andar mais rápido" ou "desviar de mais longe", os parâmetros relevantes são:

- Velocidade de cruzeiro: `src/decision.c`, `dirigir_em_direcao_a(pose, alvo_x, alvo_y, 25.0, ...)` — o `25.0`.
- Sensibilidade de detecção de obstáculo: `src/perception.c:26` (`us_sensors[1] > 350 || ...`).
- Distância do alvo: `main.c`, `alvo_x = 1500.0`.

Qualquer mudança precisa repetir os passos 2 e 3 (rebuild + redeploy) antes de rodar de novo.
