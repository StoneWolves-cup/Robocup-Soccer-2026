# Defesa

No corpo do goleiro, selecione **Estratégia > Defesa** na IHM. O rumo de
referência e o controlador fuzzy PID do BNO permanecem os mesmos do atacante.
O robô mantém esse rumo durante a busca da linha, mesmo sem detectar a bola.

Com o LDR inferior no verde, o robô procura a linha branca: recua enquanto o
ultrassônico traseiro (ECHO3) indica mais de 30 cm. Se a distância for menor
ou igual a 30 cm, passa a andar para frente até o LDR inferior detectar branco,
mesmo que a distância volte a aumentar. A leitura branca tem prioridade sobre
o ultrassônico e encerra imediatamente a busca.

Com o LDR inferior no branco, o robô acompanha a direção da bola apenas por
translação lateral; o BNO ainda pode comandar rotação para manter o rumo. A
zona morta lateral é de 5° em torno dos eixos frontal e traseiro. Sem bola,
o robô para. Se o LDR inferior voltar ao verde enquanto o robô se desloca
lateralmente, ele inverte o sentido lateral por 600 ms, mesmo que volte a
detectar branco durante esse intervalo. Depois, verifica novamente a linha e,
se estiver no verde, a distância traseira. Se estava parado quando perdeu a
linha, começa diretamente a busca.

Na busca, o ultrassônico traseiro precisa fornecer leituras válidas e recentes;
na linha, é necessário IR válido e recente. Durante a recuperação lateral,
o IR e o ultrassônico não são necessários, mas o BNO precisa estar válido
e recente em todos os movimentos. Por segurança, o robô para e interrompe
a recuperação se o BNO expirar, se o LDR central detectar branco ou se
um LDR lateral detectar branco no sentido do deslocamento lateral. Ajuste os
limiares LDR na IHM e confirme no campo os sentidos de movimento, a posição dos
sensores e a zona morta antes de operar. A distância ultrassônica é medida até
o obstáculo refletor, não até a linha branca.
