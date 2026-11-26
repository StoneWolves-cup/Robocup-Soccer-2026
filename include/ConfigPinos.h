#ifndef CONFIG_PINOS_H
#define CONFIG_PINOS_H

#include <stdint.h>

namespace ConfigPinos {
    // Mapa central de GPIO: alteracoes de placa devem ser feitas aqui para
    // manter drivers e firmwares consistentes com a ligacao eletrica.
    namespace Geral {
        namespace UART1 {
            constexpr int RX = 18;
            constexpr int TX = 17;
        }  // namespace UART1
    }  // namespace Geral

    namespace Body {
        namespace Kicker {
            // Definir todos os GPIOs antes de habilitar o kicker.
            constexpr int IN_ESQUERDA = 41;
            constexpr int IN_DIREITA = 40;
            // Fim de curso unificado: entrada com INPUT_PULLDOWN, ativa em HIGH.
            constexpr int FIM_CURSO = 20;
        }  // namespace Kicker

        namespace Motores {
            constexpr uint8_t M1_PWM = 4;
            constexpr uint8_t M1_IN1 = 6;
            constexpr uint8_t M1_IN2 = 5;
            constexpr uint8_t M2_PWM = 37;
            constexpr uint8_t M2_IN1 = 8;
            constexpr uint8_t M2_IN2 = 38;
            constexpr uint8_t M3_PWM = 16;
            constexpr uint8_t M3_IN1 = 7;
            constexpr uint8_t M3_IN2 = 15;
            constexpr uint8_t M4_PWM = 9;
            constexpr uint8_t M4_IN1 = 3;
            constexpr uint8_t M4_IN2 = 46;
        }  // namespace Motores

        namespace IHM {
            namespace OLED {
                constexpr uint8_t SDA = 21;
                constexpr uint8_t SCL = 47;
            }  // namespace OLED

            namespace Botoes {
                constexpr uint8_t SELECIONAR = 1;
                constexpr uint8_t MOVER = 42;
                constexpr uint8_t VOLTAR = 2;
            }  // namespace Botoes
        }  // namespace IHM

        namespace Bateria {
            constexpr uint8_t VBAT_ADC = 19;
            constexpr uint8_t CORRENTE_ADC = 35;
        }  // namespace Bateria

        namespace LDR {
            constexpr uint8_t MUX_SINAL = 14;
            constexpr uint8_t MUX_S0 = 10;
            constexpr uint8_t MUX_S1 = 11;
            constexpr uint8_t MUX_S2 = 12;
            constexpr uint8_t MUX_S3 = 13;
        }  // namespace LDR
    }  // namespace Body

    namespace Head {
        namespace BNO055 {
            constexpr uint8_t SDA = 8;
            constexpr uint8_t SCL = 9;
        }  // namespace BNO055

        namespace Ultrassonicos {
            constexpr uint8_t TRIG = 10;
            constexpr uint8_t ECHO[4] = {11, 12, 13, 14};
        }  // namespace Ultrassonicos

        namespace TSSP {
        // Ordem dos sensores na coroa, do índice 0 em diante no sentido horário.
            constexpr uint8_t PINOS[16] = {
                 //42, 41, 40, 39, 38, 48, 47, 21, 16, 15, 7, 6, 5, 4, 2
                1, 2, 4, 5, 6, 7, 15, 16, 21, 47, 48, 38, 39, 40, 41, 42
            };
        }  // namespace TSSP
    }  // namespace Head

    namespace Teste {
        namespace LDR8 {
            constexpr uint8_t MUX_SINAL = 34;
            constexpr uint8_t MUX_S0 = 25;
            constexpr uint8_t MUX_S1 = 26;
            constexpr uint8_t MUX_S2 = 27;
        }  // namespace LDR8

        namespace TSSP2 {
            constexpr uint8_t SENSOR1 = 8;
            constexpr uint8_t SENSOR2 = 14;
        }  // namespace TSSP2

        namespace Solenoides {
            constexpr uint8_t FIM_CURSO = 20;
            constexpr uint8_t RELE1 = 40;
            constexpr uint8_t RELE2 = 41;
        }  // namespace Solenoides
    }  // namespace Teste
}  // namespace ConfigPinos

#endif
