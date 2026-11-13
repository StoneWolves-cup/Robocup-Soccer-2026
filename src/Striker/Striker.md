# Perseguição da bola

O controlador do corpo do atacante usa a direção assinada enviada pelo anel IR:
0° é à frente, ângulos positivos e negativos indicam lados opostos, e ±180° é
atrás. Para decidir o modo, converte a leitura para 0°–360°: o setor frontal é
330°–360° ou 0°–30°. Qualquer ângulo fora desse intervalo ativa o modo curvo;
por exemplo, 270° (−90° no quadro IR) deve produzir `modo=curva`.

O rumo desejado do BNO é definido uma vez, na primeira leitura válida, e fica
fixo durante a perseguição. O robô não gira para acompanhar a bola: a manobra
em “J” é feita pela translação omnidirecional. Para isso, o vetor vai para trás
e para o lado indicado pelo ângulo IR, contornando a bola; quando a direção da
bola se aproxima do centro, o vetor curva gradualmente para frente. Por exemplo,
uma leitura de 270° (−90°) produz movimento para trás e para a direita; a
leitura IR seguinte recalcula a curva. Em 0°, o vetor aponta para frente, para
conduzir a bola ao gol.

O movimento varia de forma contínua: em ±30° aponta para trás; em ±90° fica a
45° do eixo traseiro; em ±180° chega a 45° do eixo traseiro pelo lado oposto; e
em 0° aponta para frente. Cada quadro IR válido atualiza o vetor, e quadros
expirados continuam bloqueando a perseguição. A mesma trajetória de translação
é usada com ou sem BNO; o BNO, quando habilitado, apenas mantém o rumo inicial.

## Limitações da manobra

O IR informa direção, não distância nem posição da bola. Portanto, a trajetória
em “J” é reativa e não pode garantir um ponto métrico exato de captura. A bola
deve permanecer visível no anel IR para o robô recalcular o vetor durante a
manobra.

## Contorno indicado pelo LDR e ultrassônicos

Se o vetor IR for predominantemente lateral e o LDR indicar branco no lado para
o qual o robô iria, o corpo só tenta o contorno quando há uma leitura
ultrassônica válida e recente abaixo de 10 cm: frente (ECHO4/GPIO14) aplica
−45° ao ângulo IR; traseira (ECHO3/GPIO13) aplica +45°. Se ambas estiverem
abaixo de 10 cm, nenhuma estiver próxima, ou a leitura estiver inválida ou
expirada, permanece a fuga LDR original. O contorno também é recusado quando o
LDR central indica branco ou o vetor alternativo ainda é bloqueado pelas
regiões LDR.