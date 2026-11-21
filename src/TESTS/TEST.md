# Teste de seguimento da bola por TSSP

Este teste usa dois ESP32-S3 e dois firmwares independentes:

- `pio run -e tssp-follow-emissor`: emissor da coroa TSSP.
- `pio run -e tssp-follow-receptor`: receptor que movimenta apenas os motores
  na direção da bola.

Grave cada firmware no ESP correspondente. Ligue TX GPIO17 do emissor ao RX
GPIO18 do receptor e conecte os GNDs. Ambos usam UART1, 460800 baud, 8N1.

O receptor converte o ângulo em movimento omnidirecional sem rotação e escala
a potência pela confiança, limitada a 30%. Os motores param quando um quadro
válido indica ausência da bola ou quando nenhum quadro válido chega por 100 ms.
Confirme a pinagem dos sensores e o sentido de cada motor antes do teste físico.

# Teste isolado do ultrassônico

O firmware `ultrassom-test` lê apenas um sensor ultrassônico: TRIG no GPIO10
e ECHO no GPIO11. Compile e grave com `pio run -e ultrassom-test -t upload`;
abra o monitor com `pio device monitor -e ultrassom-test`.

# Teste LDR com limiares fixos

O firmware `ldr-fixed-test` usa os 15 LDRs no CD4067BM: SIG no GPIO14 e S0-S3
nos GPIOs 10-13. O centro usa os limiares fixos de C0-C7/C14 e considera a
linha branca detectada quando pelo menos 2 dos 9 sensores centrais indicam
branco. As regiões Esquerda, Direita e Inferior usam os limiares 3332, 1304 e
2312, respectivamente. Branco é uma leitura acima do limiar e a histerese é
80 unidades ADC.

Compile e grave com `pio run -e ldr-fixed-test -t upload`, e abra o monitor com
`pio device monitor -e ldr-fixed-test`. Esta variante não usa `w/g`; os valores
podem ser ajustados nas constantes de `TEST_LDR_FIXED.cpp`. O teste original,
`pio run -e ldr-test`, continua disponível separadamente. Ambos são
ferramentas de bancada com constantes próprias e ainda não incluem a região
Superior (frente) usada pelo firmware do corpo.

## Seleção de limiares no robô

Na IHM do firmware do corpo, entre em **Testes > LDR**. Use MEXER para
escolher **LIMIAR 1** (valores de `ConfigLDR::Ataque`) ou **LIMIAR 2**
(valores de `ConfigLDR::Defesa`) e pressione SELECIONAR. A escolha restaura
os valores padrão de todas as regiões, incluindo os sete sensores centrais,
e é aplicada imediatamente. O perfil escolhido é independente da estratégia
de ataque ou defesa e permanece selecionado após reiniciar.

A região **Superior (FRT)** representa a frente do robô e usa a média de dois
LDRs, como as demais regiões periféricas: C14 e C0 (C0 foi retirado do anel
central, que agora usa C1-C7 e exige 2 de 7 sensores brancos). Detectar a
frente faz o robô recuar imediatamente, sem depender da memória da última
correção. Se o sensor frontal físico não for o C0, troque-o em
`CHANNELS_CENTER` e `CHANNELS_REGIONS` (`LDRFixed.cpp`) e reordene os limiares
em `ConfigLDR.h`. O limiar padrão de FRT é a média dos antigos limiares de C0 e
C14 e deve ser recalibrado no campo.

Na mesma tela é possível ajustar ESQ, DIR, INF e FRT em passos de 5: selecione a
região, use SELECIONAR para aumentar ou MEXER para diminuir e VOLTAR para
sair. Os ajustes são salvos para o perfil ativo; selecionar novamente um
limiar restaura seus valores padrão. Após apagar a flash, o padrão inicial é
LIMIAR 1. Se a tela mostrar `ERRO NO NVS`, a alteração está aplicada em
memória, mas pode não ser preservada após reiniciar; consulte o monitor serial.

## Comportamento da defesa

Com a estratégia de defesa ativa, o ultrassônico traseiro (ECHO4) e o BNO
precisam fornecer leituras válidas e recentes, e o LDR deve estar habilitado.
O robô controla a distância traseira em 42,5 cm, com zona morta de ±2 cm. No
posicionamento inicial, também exige que o LDR inferior detecte branco. Os
comandos longitudinais da defesa são invertidos em relação ao eixo `y` geral
para corresponder ao sentido observado no robô. Se a distância já estiver na
zona morta e o inferior ainda estiver verde, busca o branco lentamente para
trás; depois corrige a distância e só inicia o acompanhamento quando ambas as
condições forem satisfeitas.

Após o posicionamento inicial, a distância continua sendo corrigida pelo
ultrassônico. Com bola válida no IR, o robô acompanha sua direção apenas pelo
eixo horizontal (`x > 0` esquerda, `x < 0` direita), proporcionalmente ao seno
do ângulo e com zona morta de 10°. A rotação continua vindo exclusivamente da
correção de rumo do BNO, nunca do ângulo da bola. Sem bola válida, mantém a
distância e o rumo, sem deslocamento lateral. Uma leitura traseira inválida ou
expirada bloqueia os motores.

## Kicker duplo

O firmware de bancada `pio run -e kicker-test` roda no Body e recebe os
ultrassônicos pelo firmware Head conectado à UART1 (TX 17 e RX 18, GND comum).
Para gravar, use `pio run -e kicker-test -t upload`; abra o monitor com
`pio device monitor -e kicker-test`. Este teste não usa a IHM nem exige IR e
BNO: com os GPIOs configurados, pode chutar automaticamente ao pressionar um
fim de curso. Faça o teste em área segura, sem pessoas na direção do chute.

O kicker fica desabilitado enquanto os três GPIOs em `ConfigPinos::Body::Kicker`
(`IN_ESQUERDA`, `IN_DIREITA` e `FIM_CURSO`) estiverem indefinidos. Antes de usá-lo,
defina pinos disponíveis e distintos, confira conflitos com os demais componentes e
valide as ligações em bancada: as entradas IN da CDI são ativas em HIGH e o fim de
curso unificado (um único pino para os dois fins de curso) é ativo em HIGH com
`INPUT_PULLDOWN`. O firmware mantém as saídas em LOW quando desabilitadas;
use pull-downs externos nas entradas da CDI para evitar acionamentos durante
boot ou reset, antes da configuração dos GPIOs.

Em ataque ou defesa, o fim de curso unificado acionado permite selecionar o
chute. Os ultrassônicos são U1/direita, U2/frente, U3/esquerda e U4/trás.
Com frente acima de 80 cm, esquerda abaixo de 65 cm seleciona o kicker
esquerdo, direita abaixo de 65 cm o direito; na faixa central ou quando ambas
as laterais estiverem abaixo de 65 cm, não dispara. Com frente até 80 cm,
a lateral com distância maior seleciona o kicker oposto; empate bloqueia.
As três leituras devem ser válidas e recentes, e a comunicação do Head deve
estar ativa. Não há inferência de um sensor a partir do outro.

Bateria sem leitura válida ou abaixo de 40% bloqueia o chute; perda de
condições durante o pulso desliga as saídas. Cada pulso tem duração nominal
de 400 ms, encerrado por temporizador independente do loop, seguido por
600 ms de cooldown global. O temporizador é de software e não substitui uma
proteção elétrica independente contra pulso prolongado. Enquanto um fim de
curso continuar pressionado, um novo chute pode ocorrer após o cooldown se as
condições ainda permitirem.

## Diagnóstico de reinicialização e teste seguro da fuga (ataque)

O Body imprime `Motivo do reset: <codigo> (<descricao>)` no monitor serial a
460800 baud. Registre essa linha após cada reinicialização:
`brownout/tensao baixa` aponta para investigar a alimentação sob carga;
`watchdog` ou `panic/crash` requer investigar o caminho de firmware indicado
pelo log. Não desative o brownout para esconder reinicializações. Se houver
brownout, confira bateria, regulador, conexões, driver e queda de tensão durante
o acionamento dos motores.

Teste primeiro com o robô suspenso, confirmando o sentido dos motores e que a
detecção de linha aciona o vetor apropriado. Depois, em área delimitada e com
operador pronto para desligar, verifique detecção intermitente, múltiplas
regiões e detecção persistente. O último caso deve parar os motores após 1 s;
a fuga só rearma quando a linha fica limpa por 120 ms.

Ao detectar a frente no ataque, a fuga para todos os motores por 50 ms e
comanda recuo reto durante os 250 ms seguintes, sem perseguir a bola. Para
voltar à perseguição, também é necessário que todas as regiões estejam limpas
por 120 ms. Se detectar a região inferior durante a sequência, cancela a fuga
frontal e reage à nova linha. As outras fugas mantêm 400 ms de linha limpa.
A segurança de 1 s conta a detecção contínua e interrompe a fuga se a linha
não desaparecer. Os tempos estão em `FugaLinha.cpp`.

## Curva J e intensidade do IR

O desvio máximo do J varia de 6° a 30° conforme a proximidade estimada: quanto
maior o índice, maior o desvio. Em 130° a direção de movimento é para trás;
acima disso, o desvio cresce quadraticamente. Entre 90° e 130° a curva é
interpolada para evitar um salto de direção em 90°. A captura frontal até 45°
permanece igual.

A estimativa combina com pesos iguais o maior percentual de atividade dos TSSP
e o número de sensores com atividade acima de 2%. Cada indicador é limitado
entre 0 e 1: a atividade usa os pontos 2% e 25%, e a contagem usa 1 e 6 sensores.
O resultado é suavizado a cada pacote IR válido (fator 0,2) e reiniciado quando
a bola é perdida ou a telemetria expira. Os parâmetros estão em
`include/ProximidadeIR.h`; os limites do J, em `include/MovimentoBola.h`.

Em **Telemetria > Infra IR** e no monitor serial aparecem `LONGE`, `MEDIA` ou
`PERTO`, o índice relativo, o máximo e a contagem. `conf` no serial é a
concentração direcional, não distância. **Esses limiares são provisórios, não
representam distância em centímetros e dependem da bola, orientação, sensores
e ambiente.** Registre leituras em várias distâncias e ângulos para calibrá-los.
Teste primeiro com o robô suspenso e depois em área delimitada.

O teste local dos cálculos pode ser executado com
`g++ -std=c++11 -I include src/TESTS/TEST_PROXIMIDADE_IR/test.cpp -o /tmp/test-proximidade-ir && /tmp/test-proximidade-ir`.

## Avanço prioritário da defesa

Na defesa, U4 <= 25 cm com Superior ou Centro branco ativa um avanço frontal
que permanece prioritário até U4 atingir 40 cm. Enquanto o estado estiver
ativo, correções das regiões LDR são ignoradas, mas o controle lateral normal
da defesa e a correção de rumo continuam. Leitura inválida de U4 encerra o
estado e para o movimento com segurança.

O teste local dessa decisão pode ser executado com
`g++ -std=c++11 -I include src/TESTS/TEST_MOVIMENTO_DEFESA/test.cpp -o /tmp/test-movimento-defesa && /tmp/test-movimento-defesa`.

Teste do kicker (nativo, via PlatformIO): `pio run -e kicker-test && .pio/build/kicker-test/program`.

## Correção de rumo e teste BNO-MOVE

Ataque e defesa usam PID fuzzy para corrigir o heading do BNO; essa correção
permanece ativa durante a fuga da linha (ataque) e durante todo o movimento da
defesa, mesmo sem detecção da bola. Se a mistura
de motores saturar, a cinemática preserva a rotação do BNO e reduz a translação.
O teste BNO-MOVE continua usando PID convencional, com integral independente. Os
ganhos Kp, Ki e Kd ajustados em **Testes > BNO-MOVE** são compartilhados
pelos dois controladores e salvos no NVS. Os padrões são Kp 0,0044,
Ki 0,0000022 e Kd 0,00012; valores já salvos prevalecem sobre os padrões.
O fuzzy ajusta esses ganhos conforme o erro angular e a velocidade do gyro.

Esses valores são um ponto de partida, não uma sintonia garantida. Verifique a
calibração do BNO/magnetômetro e teste primeiro com o robô suspenso, depois em
campo, observando oscilações e interferência dos motores.
