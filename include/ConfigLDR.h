#ifndef CONFIG_LDR_H
#define CONFIG_LDR_H

#include <stdint.h>

namespace ConfigLDR {
    // Perfil completo por estrategia: limiares perifericos editaveis na IHM,
    // nove limiares centrais independentes e histerese do anel central.
    struct Limiares {
        uint16_t esquerda;
        uint16_t direita;
        uint16_t inferior;
        uint16_t centro[9];  // C0, C1, C2, C3, C4, C5, C6, C7, C14
        uint16_t histereseCentro;
    };

    // Cada perfil e aplicado ao selecionar a estrategia; ajuste em campo.
    constexpr Limiares Ataque = {
        3332, 1324, 2312,
        {2619, 2890, 3699, 1936, 2045, 1776, 2924, 1874, 3647},
        20
    };

    constexpr Limiares Defesa = {
        178, 2189, 305,
        {406, 248, 379, 252, 349, 85, 297, 113, 140},
        20
    };
}  // namespace ConfigLDR

#endif
