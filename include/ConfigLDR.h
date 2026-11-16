#ifndef CONFIG_LDR_H
#define CONFIG_LDR_H

#include <stdint.h>

namespace ConfigLDR {
struct Limiares {
    uint16_t esquerda;
    uint16_t direita;
    uint16_t inferior;
    uint16_t centro[9];  // C0, C1, C2, C3, C4, C5, C6, C7, C14
    uint16_t histereseCentro;
};

constexpr Limiares Ataque = {
    3332, 1304, 2312,
    {2619, 2890, 3699, 1936, 2045, 1776, 2924, 1874, 3647},
    80};

constexpr Limiares Defesa = {
    287, 353, 173,
    {409, 160, 381, 260, 292, 72, 234, 126, 166},
    20};
}  // namespace ConfigLDR

#endif
