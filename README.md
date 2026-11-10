
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

<p>O Head envia os dados dos 16 sensores TSSP ao Body pela UART1 a 115200 baud,
8N1. Ligue TX GPIO17 do Head a RX GPIO18 do Body e una os GNDs. O Body acende
o RGB em vermelho enquanto aguarda dados validos e em verde quando a UART esta
ativa. Conecte o BNO055 ao Head em SDA GPIO8 e SCL GPIO9. O Head envia rumo e
velocidade angular junto com os dados da bola; o Body segue a direcao detectada
e corrige a rotacao para manter o rumo inicial. Sem dados recentes do BNO055,
os motores permanecem parados. As leituras e a direcao da bola aparecem no
monitor serial do Head; rumo e correcao aparecem no monitor do Body.</p>
<p>O Body usa PID de rumo (Kp=0.0015, Ki=0.0001, Kd=0.0020), com integral
limitada para evitar windup. Esses ganhos devem ser ajustados com testes
controlados no robo.</p>

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
