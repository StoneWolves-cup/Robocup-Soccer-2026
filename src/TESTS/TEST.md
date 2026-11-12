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
`pio run -e ldr-test`, continua disponível separadamente.