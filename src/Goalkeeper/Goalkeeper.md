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
e recente em todos os movimentos.

Se um LDR lateral detectar branco, o robô interrompe a estratégia atual e se
desloca para o lado oposto enquanto a detecção persistir: branco à esquerda
provoca movimento à direita e vice-versa.

Se ambos detectarem branco, o robô avança por 400 ms e para para aguardar um
novo pacote ultrassônico. Compara os laterais esquerdo (ECHO1/GPIO11) e direito
(ECHO2/GPIO12) e se desloca por 400 ms para o lado oposto à menor distância.
Ele para enquanto qualquer leitura lateral estiver inválida ou expirada ou
enquanto as duas distâncias forem iguais. Depois, procura o branco inferior
enquanto se alinha lateralmente à bola. O comando de recuo corresponde a 50%
da potência total dos motores sem movimento lateral ou correção de rumo;
a combinação dos movimentos pode alterar a potência efetiva. O limite traseiro
de 30 cm continua ativo. Quando o branco inferior for encontrado, retoma a
defesa normal. Para não repetir continuamente a manobra, ela só é rearmada
quando ambos os LDRs laterais deixarem de detectar branco; uma única detecção
lateral continua tendo prioridade durante a busca.

As correções laterais têm prioridade sobre a recuperação de 600 ms. O LDR
central não é usado pela estratégia de defesa. Por segurança, o robô para se
o BNO expirar. Ajuste os limiares LDR na IHM e confirme no campo os sentidos
de movimento, a posição dos sensores e a zona morta antes de operar. A distância
ultrassônica é medida até o obstáculo refletor, não até a linha branca.

Os limiares padrão da defesa e do ataque ficam separados em
[ConfigLDR.h](../../include/ConfigLDR.h). Na IHM, acesse **Checklist > LDR**:
use MEXER para escolher a região, SEL para iniciar o ajuste, SEL para aumentar
e MEXER para diminuir de 5 em 5. VOLTAR retorna ao checklist. Os ajustes são
salvos separadamente para cada robô e prevalecem sobre os padrões do arquivo
após reiniciar, inclusive depois de gravar o firmware. Os nove limiares
centrais e sua histerese são ajustados no arquivo e exigem nova gravação do
firmware. A histerese das regiões laterais e inferior permanece em 80 ADC.
