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

Na mesma tela é possível ajustar ESQ, DIR, INF e FRT em passos de 10. Selecione
a região e use SELECIONAR para aumentar ou MEXER para diminuir. A opção CENTRO
abre S1-S7; selecione um sensor para ver e ajustar seu limiar também em passos
de 10. Durante o ajuste, VOLTAR retorna à lista de sensores. Os ajustes são
salvos para o perfil ativo; selecionar novamente um limiar restaura seus valores
padrão. Após apagar a flash, o padrão inicial é LIMIAR 1. Se a tela mostrar
`ERRO NO NVS`, a alteração está aplicada em memória, mas pode não ser preservada
após reiniciar; consulte o monitor serial.

## Comportamento da defesa

Com a estratégia de defesa ativa, o ultrassônico traseiro (ECHO4) e o BNO
precisam fornecer leituras válidas e recentes. O robô controla a distância
traseira em 40 cm, com zona morta de ±3 cm.

Com bola válida no IR, um PID acompanha sua direção pelo eixo horizontal
(`x > 0` esquerda, `x < 0` direita), com zona morta de ±2°. O PID usa novos
quadros IR e é reiniciado ao perder a bola, alcançar a zona morta, interromper
a defesa ou quando o avanço e os limites laterais substituem seu comando.
A rotação continua vindo exclusivamente do BNO. Sem bola, a defesa usa U1 para
retornar ao centro; uma leitura traseira inválida ou expirada bloqueia os motores.

Quando a bola permanece detectada, classificada como `MEDIA DISTANTE`, `MEDIA`
ou `PERTO` e dentro do limiar angular adaptativo por 1,2 s, a defesa faz um
avanço apontado para a bola durante até 2 s, seguido de cooldown de 500 ms.
O avanço é interrompido se a bola for perdida ou passar a `MUITO LONGE`. O
limiar angular varia de 3° a 14° conforme a proximidade e é independente da
zona morta do PID lateral.
O avanço mantém a correção de rumo do BNO e está sujeito aos limites laterais.
Os ganhos do PID lateral são valores iniciais e devem ser ajustados com o robô
suspenso e depois em campo; confirme também o sentido dos motores e do IR.
Em `src/Body/main.cpp`, `HABILITAR_AVANCO_BOLA_DEFESA` habilita esse avanço
quando `true`; com `false`, a defesa continua funcionando sem o avanço
temporizado. A alteração exige recompilar o firmware.

## Kicker duplo

O firmware de bancada `pio run -e kicker-test` alterna o relé direito a cada
segundo, sem consultar os sensores. Ele usa o mesmo GPIO 40 (`IN_DIREITA`)
e a mesma polaridade ativa em LOW do firmware Body. Para gravar, use
`pio run -e kicker-test -t upload`; o monitor serial usa 115200 baud.
O firmware Body também permite disparos manuais pela IHM.

No Body, selecione **Telemetria > Kicker Auto** para executar o teste
automático com os motores parados. A tela mostra o estado do fim de curso, as
distâncias direita, frontal e esquerda e os estados da comunicação e da bateria.
O teste usa as mesmas condições e o mesmo acionamento do ataque descritos
abaixo: pressionar o fim de curso pode acionar os relés imediatamente e
repetir o disparo após cada cooldown enquanto ele continuar pressionado.
Pressione **Voltar** para sair e interromper um pulso automático em andamento.
O modo depende do firmware Body e não funciona no ambiente isolado `ihm-test`.
Faça o teste em bancada, em área delimitada e com o mecanismo livre de pessoas.

Em **Telemetria > Kicker Fim Curso**, o fim de curso aciona os dois relés
sem depender dos ultrassônicos ou da comunicação com o Head. A bateria precisa
ter leitura válida de pelo menos 40%. O pulso e o cooldown são os mesmos do
ataque; enquanto o fim de curso estiver pressionado, o disparo se repete após
o cooldown. A tela mostra o estado do fim de curso e da bateria. **Voltar**
desativa o teste e interrompe o pulso automático. Esse modo também exige o
firmware Body e deve ser usado em bancada com os motores parados.

Confira os três GPIOs em `ConfigPinos::Body::Kicker` (`IN_ESQUERDA`,
`IN_DIREITA` e `FIM_CURSO`), possíveis conflitos e as ligações em bancada.
As saídas de relé são ativas em LOW e o fim de curso unificado é ativo em HIGH
com `INPUT_PULLDOWN`. Garanta por hardware que os relés permaneçam desligados
durante o boot ou reset, antes da configuração dos GPIOs.
No Body, o nível HIGH é preparado antes de habilitar as saídas GPIO. Isso não
controla os pinos antes de o firmware executar: verifique com o fabricante do
módulo um circuito externo que mantenha cada entrada IN no nível inativo
durante boot, reset e perda de alimentação do ESP32, sem aplicar tensão acima
do limite permitido aos GPIOs.

No ataque, o fim de curso unificado acionado permite selecionar o
chute. Os ultrassônicos são U1/direita, U2/frente, U3/esquerda e U4/trás.
Quando somente o ultrassom esquerdo estiver até 30 cm, seleciona o kicker
esquerdo. Quando somente o direito estiver até 30 cm, seleciona o direito.
Se nenhum ou ambos estiverem dentro desse limite, os dois kickers acionam juntos.
Leituras fora da faixa válida do sensor bloqueiam o disparo.
As três leituras devem ser válidas e recentes, e a comunicação do Head deve
estar ativa. Não há inferência de um sensor a partir do outro.

Na estratégia de ataque, temporariamente o acionamento automático usa somente
U1/direita: com leitura válida acima de 100 cm, dispara o kicker esquerdo, sem
exigir o fim de curso. Os outros ultrassônicos não participam dessa decisão. As
proteções de bateria, comunicação, validade/atualidade da leitura e cooldown
continuam ativas.

Bateria sem leitura válida ou abaixo de 40% bloqueia o chute automático; perda
de ataque, comunicação, bateria válida ou ultrassônicos válidos durante o pulso
desliga as saídas. O disparo manual pela IHM ignora esses requisitos, mas respeita
o pulso em andamento e o cooldown. Cada pulso tem duração nominal
de 400 ms, encerrado por temporizador independente do loop, seguido por
600 ms de cooldown global, inclusive se o pulso automático for interrompido
antes do fim. O temporizador é de software e não substitui uma
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

O desvio máximo do J varia gradualmente de 6° (bola longe), 12° (proximidade
média) a 24° (bola perto). O movimento começa a se afastar da direção da bola
a partir de 70°; a curva cresce suavemente até 180°. O lado da curva é mantido
quando a bola está quase atrás, evitando que o robô troque de lado por pequenas
variações do ângulo IR.

A estimativa combina com pesos iguais a atividade máxima dos TSSP e a contagem
de sensores acima do limiar de 1%. Cada indicador é limitado entre 0 e 1: a
atividade usa os pontos 6,7% e 24%, e a contagem usa 1 e 3 sensores. O resultado
é suavizado a cada pacote IR válido (fator 0,3) e reiniciado quando a bola é
perdida ou a telemetria expira. Os parâmetros estão em `include/ProximidadeIR.h`;
os limites do J, em `include/MovimentoBola.h`.

Em **Telemetria > Infra IR** e no monitor serial aparecem `MUITO LONGE`,
`MEDIA DISTANTE`, `MEDIA` ou `PERTO`, o índice relativo, o máximo e a contagem.
`conf` no serial é a
concentração direcional, não distância. **Esses limiares são provisórios, não
representam distância em centímetros e dependem da bola, orientação, sensores
e ambiente.** Registre leituras em várias distâncias e ângulos para calibrá-los.
Teste primeiro com o robô suspenso e depois em área delimitada.

## Avanço prioritário da defesa

Na defesa, U4 <= 25 cm com Superior ou Centro branco ativa um avanço frontal
que permanece prioritário até U4 atingir 40 cm. Enquanto o estado estiver
ativo, correções das regiões LDR são ignoradas, mas o controle lateral normal
da defesa e a correção de rumo continuam. Leitura inválida de U4 encerra o
estado e para o movimento com segurança.

Teste primeiro com o robô suspenso; a automação só deve ser usada em
conformidade com o regulamento vigente da RoboCup Soccer Infrared.

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
O controlador das estratégias calcula a correção apenas ao receber um novo
quadro BNO e mantém a última saída entre quadros; assim o tempo da integral
acompanha as amostras, não a velocidade variável do loop.

Esses valores são um ponto de partida, não uma sintonia garantida. Verifique a
calibração do BNO/magnetômetro e teste primeiro com o robô suspenso, depois em
campo, observando oscilações e interferência dos motores.
