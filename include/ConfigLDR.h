#ifndef CONFIG_LDR_H
#define CONFIG_LDR_H

#include <stdint.h>

namespace ConfigLDR {
    // Perfil completo por estrategia: limiares perifericos editaveis na IHM
    // (superior = frente, media de C14 e C0), sete limiares centrais
    // independentes e histerese do anel central.
    struct Limiares {
        uint16_t esquerda;
        uint16_t direita;
        uint16_t inferior;
        uint16_t superior;
        uint16_t centro[7];  // C1, C2, C3, C4, C5, C6, C7
        uint16_t histereseCentro;
    };

    // Cada perfil e aplicado ao selecionar a estrategia; ajuste em campo.
    // O limiar superior inicial e a media dos antigos limiares de C0 e C14.
    constexpr Limiares Ataque = {
        102, 2100, 142, 142,
        {210, 140, 210, 128, 230, 60, 208},
        20
    };

    constexpr Limiares Defesa = {
        178, 2189, 305, 273,
        {248, 379, 252, 349, 85, 297, 113},
        20
    };
}  // namespace ConfigLDR

#endif
