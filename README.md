
<h1 align=center>Robocup-Soccer-2026</h1>

<p align="center">
  <img src="https://img.shields.io/badge/RoboCup-2026-blue">
  <img src="https://img.shields.io/badge/Category-Soccer_Infrared-green">
  <img src="https://img.shields.io/badge/Country-Brazil-yellow">
  <img src="https://img.shields.io/badge/Team-Stone_Wolves-orange">
</p>

<hr>

<h2 align=center>Team Name</h2>
<p align=center width="150"> 
Stone Wolves <br><img src="Images/Logo.png" width ="250">
</p>

<hr>

<h2 align=center>Localization:</h2>
<p align=center>
Ribeirão Preto, São Paulo, Brazil
</p>

<hr>

<h2 align=center>Robot Images</h2>

<p align=center>
  <img src="Images/Robô_Frente.png" width="300">
  <br>
  Front view of the robot modeled in Fusion
</p>

<p align=center>
  <img src="Images/Robô_Tras.png" width="300">
  <br>
  Back view of the robot modeled in Fusion
</p>

<p align=center>
  <img src="Images/Robô_Esquerda.png" width="300">
  <br>
  Left-side view of the robot modeled in Fusion
</p>

<p align=center>
  <img src="Images/Robô_Direita.png" width="300">
  <br>
  Right-side view of the robot modeled in Fusion
</p>

<hr>

<h2 align=center>Team Members</h2>

<ul align=center>
  <li>Cauã Moraes</li>
  <li>Felipe Siqueira</li>
  <li>Luana Bazan</li>
  <li>Luiz Henrique</li>
  <li>Nicoly Aguiar</li>
  <li>Raul Furlan</li>
  <li>Samuel Brito</li>
</ul>

<hr>

<h2 align=center>Coach</h2>

<p align=center>
Yuri Rezende
</p>

<hr>

<h2 align=center>Team Photo</h2>

<p align=center>
  <img src="Images/Foto_Equipe.jpeg" width="300">
  <br>
  Team Picture
</p>

<hr>

<h2 align=center>About the Project</h2>

<p align=center>
Stone Wolves is a robotics soccer team created to participate in RoboCup Soccer Infrared 2026.
Our objective is to develop autonomous robots capable of playing soccer through programming, mechanics, and electronics.
</p>

<hr>

<h2 align=center>Technologies Used</h2>

<ul align=center>
  <li>Fusion 360</li>
  <li>Arduino IDE</li>
  <li>ESP32-S3 (UART1 entre Head e Body)</li>
</ul>

<p>O Head envia os dados dos 16 sensores TSSP e a telemetria do BNO055 ao Body
pela UART1 a 115200 baud, 8N1. A UART do Body inicializa antes da IHM. O monitor
USB dos firmwares Head e Body usa 460800 baud. Ligue TX GPIO17 do Head a RX
GPIO18 do Body e una os GNDs. O Body acende
o RGB em vermelho enquanto aguarda dados validos e em verde somente quando
recebe pacotes IR e BNO validos e recentes. O monitor serial do Body informa
por segundo a quantidade de bytes e pacotes aceitos por tipo e o ultimo erro
de protocolo. Conecte o BNO055 ao Head em SDA GPIO8 e SCL GPIO9. O Head envia rumo e
velocidade angular junto com os dados da bola; o Body segue a direcao detectada
e corrige a rotacao para manter o rumo inicial. Sem dados recentes do BNO055,
os motores permanecem parados. As leituras e a direcao da bola aparecem no
monitor serial do Head; rumo e correcao aparecem no monitor do Body.</p>
<p>Atualmente, a decisao de movimento do Striker usa somente IR e BNO055. Os
LDRs de linha estao temporariamente desativados: nao e necessario calibrar e
nao ha protecao contra a linha branca. Com a bola detectada, o Body move na
direcao indicada pelo IR e corrige a rotacao com o BNO. Sem bola detectada ou
sem telemetria BNO recente, os motores permanecem parados. O monitor serial
informa o motivo de bloqueio. Teste os motores com o robo suspenso ou em uma
bancada segura enquanto os LDRs estiverem desativados.</p>
<p>O Body usa FuzzyPID para corrigir o rumo com base no erro angular e na
velocidade do giroscopio. Os ganhos base atuais sao Kp=0.0054, Ki=0.0000030 e
Kd=0.0007; o ajuste fuzzy adapta esses ganhos durante a operacao. A integral
e limitada a 300 e a rotacao de correcao a 0.30. Esses valores devem ser
ajustados com testes controlados no robo.</p>

<h3>Teste de leitura percentual da bateria</h3>
<p>O teste para ESP32-S3 usa o estimador em <code>include/BatteryEstimator.h</code>
e exibe a tensão e o percentual calculado no monitor serial. Conecte o ponto
médio do divisor resistivo ao GPIO4 e o GND da bateria ao GND do ESP32.
Antes de ligar, confira os resistores e a tensão no ADC; ajuste
<code>DIVIDER_R1_OHM</code>, <code>DIVIDER_R2_OHM</code> e a curva de tensão
para a bateria utilizada.</p>
<pre><code>pio run -e bateria-test -t upload
pio device monitor -e bateria-test</code></pre>

<h3>Teste regional dos LDRs</h3>
<p>O teste usa 15 LDRs conectados aos canais C0-C14 de um CD4067BM no ESP32-S3.
Ligue SIG ao GPIO14 e S0-S3 aos GPIOs 10-13, respectivamente; o canal C15 fica
sem uso. As regiões são Centro (C0-C7 e C14), Esquerda (C12-C13), Direita
(C8-C9) e Inferior (C10-C11). Os LEDs brancos acendem automaticamente e não
precisam de controle pelo firmware. Calibre sobre branco com <code>w</code> e
depois sobre verde com <code>g</code>; <code>r</code> reinicia a calibração.
Cada LDR recebe seu limiar a partir das duas referências, e o teste compara a
média de leitura de cada região com a média dos limiares daquela região.</p>
<pre><code>pio run -e ldr-test -t upload
pio device monitor -e ldr-test</code></pre>

<h3>Teste do angulo da bola via UART</h3>
<p>O emissor lê os 16 sensores TSSP, calcula a direção da bola e envia o pacote
IR pela UART1 a 460800 baud. O receptor é um firmware separado que apenas imprime os dados
IR recebidos no monitor serial, sem depender do Body. Conecte TX GPIO17 do
emissor ao RX GPIO18 do receptor e una os GNDs. Grave
<code>uart-ir-emissor-test</code> no emissor e
<code>uart-ir-receptor-test</code> no receptor. O monitor serial do receptor
mostra as atividades dos 16 sensores, a direção e a confiança da detecção a
460800 baud. Configure também o monitor USB dos testes UART para 460800 baud.
Use no emissor os mesmos GPIOs e conexões da coroa TSSP de 16 sensores. O
receptor avisa no serial se nenhum byte chegar ou se os bytes não formarem um
pacote IR válido, e informa quando a comunicação voltar.</p>
<pre><code>pio run -e uart-ir-emissor-test -t upload
pio run -e uart-ir-receptor-test -t upload
pio device monitor -e uart-ir-receptor-test</code></pre>

<h3>Teste padrão de mensagem UART</h3>
<p>Grave <code>uart-emissor-test</code> e <code>uart-receptor-test</code> em
duas ESP32-S3. Ligue TX GPIO17 do emissor ao RX GPIO18 do receptor e una os
GNDs. A UART1 deste teste usa 115200 baud (8N1); o monitor USB permanece em
460800 baud. O receptor fica vermelho enquanto aguarda e fica verde ao receber
uma linha completa. Se passar 2,5 s sem uma linha, volta ao vermelho e informa
a falha no monitor; ao receber novamente, informa a recuperação.
<pre><code>pio run -e uart-emissor-test -t upload
pio run -e uart-receptor-test -t upload
pio device monitor -e uart-receptor-test</code></pre>

<h3>Teste de correção de rumo via UART</h3>
<p>O emissor <code>uart-bno-emissor-test</code> lê o BNO055 e envia somente o
ângulo, em graus, como um número por linha. Ele define o rumo inicial como
zero; envie <code>c</code> no monitor serial do emissor para redefinir essa
referência. O receptor <code>bno-move-test</code> calcula o PID, gira os motores
para corrigir o rumo e para os motores se perder os dados UART por mais de
300 ms. Ligue TX GPIO17 de cada placa ao RX GPIO18 da outra e una os GNDs.
No monitor serial do receptor, use <code>q/a</code> para aumentar/diminuir Kp,
<code>w/s</code> para Ki e <code>e/d</code> para Kd. Os ganhos iniciais são
Kp=0.083, Ki=0.0000035 e Kd=0.0010; os valores atuais aparecem na telemetria.</p>
<pre><code>pio run -e uart-bno-emissor-test -t upload
pio run -e bno-move-test -t upload
pio device monitor -e bno-move-test</code></pre>

<h3>Checklist de testes na IHM</h3>
<p>No menu CHECKLIST, use MEXER para navegar pelos testes e SELECIONAR para
abrir o teste ou ver o alvo PlatformIO correspondente. A tela mostra os
códigos <code>bno-test</code>, <code>tssp-16-test</code>,
<code>ultrassonicos-test</code>, <code>motores-test</code>,
<code>motor-unitario-test</code>, <code>uart-receptor-test</code>,
<code>uart-emissor-test</code> e <code>ldr-test</code>. O teste de solenoides
fica em <code>src/TESTS/TEST_SOLENOIDES/*.ino</code> e ainda não possui
ambiente PlatformIO próprio. A IHM não grava outro firmware: para executar um
teste separado, grave o ambiente correspondente pelo computador, por exemplo
<code>pio run -e motores-test -t upload</code>. Os testes de motores e
solenoides devem ser executados em bancada controlada.</p>

<hr>

<h2 align=center>Robot Video</h2>

<p align=center>
Watch our robot playing.
</p>

<p align=center>
  <a href="Images/Video.mp4">Click to Video</a>
</p>

<hr>

<p align="center">
  <img src="https://komarev.com/ghpvc/?username=StoneWolves&label=Visitors&color=blue">
</p>
