#ifndef ESPNOW_LINK_H
#define ESPNOW_LINK_H

#include <Arduino.h>

// Vinculo ESP-NOW entre os robos de ataque e defesa. O parceiro nunca e
// escolhido as cegas: com um unico MAC online a conexao e automatica, com mais
// de um o operador escolhe na telemetria e o MAC fica gravado na NVS.
namespace ESPNowLink {

enum class Papel : uint8_t { Nenhum = 0, Ataque = 1, Defesa = 2 };

enum class Estado : uint8_t {
  Procurando,     // Sem parceiro gravado e nenhum MAC online.
  EscolhaManual,  // Sem parceiro gravado e mais de um MAC online.
  Definindo,      // Parceiro gravado; repetindo o handshake ate o ACK.
  Conectado       // Handshake concluido e heartbeats recentes.
};

constexpr uint8_t MAX_CANDIDATOS = 6;

void begin();
void update();

// Habilita a escolha automatica quando houver exatamente um MAC online.
void iniciarBusca();
// Estado local anunciado ao parceiro; defOn e o sinal RoboDefOn.
void definirEstadoLocal(Papel papel, bool defOn);

Estado estado();
void macLocal(uint8_t mac[6]);
// Copia os MACs vistos recentemente; retorna a quantidade copiada.
uint8_t candidatosOnline(uint8_t macs[][6], uint8_t maximo);
// Grava o MAC escolhido na NVS e inicia o handshake.
bool selecionarMac(const uint8_t mac[6]);
bool possuiVinculo();
void macVinculo(uint8_t mac[6]);
void esquecerVinculo();

bool conectado();
// true somente com vinculo valido e recente, RoboDefOn do parceiro em false e
// parceiro que nao esteja em ataque (evita dois robos trocando para defesa).
bool defesaParceiroDesligada();

}  // namespace ESPNowLink

#endif
