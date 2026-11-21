#include <ESPNowLink.h>
#include <IRProtocol.h>
#include <Preferences.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <string.h>

namespace ESPNowLink {
namespace {

constexpr uint32_t INTERVALO_ANUNCIO_MS = 500;
constexpr uint32_t INTERVALO_PAIR_MS = 300;
constexpr uint32_t INTERVALO_STATE_MS = 100;
constexpr uint32_t TIMEOUT_CANDIDATO_MS = 3000;
constexpr uint32_t TIMEOUT_VINCULO_MS = 1000;
constexpr uint8_t BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

struct Candidato {
  uint8_t mac[6];
  uint32_t vistoEmMs;
};

portMUX_TYPE trava = portMUX_INITIALIZER_UNLOCKED;
bool iniciado = false;
bool buscaAtiva = false;
uint8_t meuMac[6] = {};
uint8_t macParceiro[6] = {};
bool temParceiro = false;
Candidato candidatos[MAX_CANDIDATOS] = {};

// Estado escrito pelo callback de recepcao (task Wi-Fi) e lido no loop.
bool parceiroConectado = false;
uint32_t ultimoPacoteParceiroMs = 0;
bool parceiroDefOn = false;
Papel parceiroPapel = Papel::Nenhum;
bool ackPendente = false;

Papel papelLocal = Papel::Nenhum;
bool defOnLocal = false;
uint16_t sequencia = 0;
uint32_t ultimoAnuncioMs = 0;
uint32_t ultimoPairMs = 0;
uint32_t ultimoStateMs = 0;

bool mesmoMac(const uint8_t *a, const uint8_t *b) {
  return memcmp(a, b, 6) == 0;
}

void enviar(const uint8_t *destino, uint8_t tipo) {
  ESPNowLinkPacket pacote{};
  pacote.magic = ESPNOW_LINK_MAGIC;
  pacote.version = ESPNOW_LINK_VERSION;
  pacote.type = tipo;
  pacote.sequence = sequencia++;
  pacote.role = static_cast<uint8_t>(papelLocal);
  pacote.defOn = defOnLocal ? 1 : 0;
  pacote.checksum = espNowLinkChecksum(pacote);
  esp_now_send(destino, reinterpret_cast<const uint8_t *>(&pacote),
               sizeof(pacote));
}

bool adicionarPeer(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) return true;
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = 0;
  peer.encrypt = false;
  return esp_now_add_peer(&peer) == ESP_OK;
}

// Chamada dentro da secao critica; reaproveita vaga expirada ou o mais antigo.
void registrarCandidato(const uint8_t *mac, uint32_t agora) {
  int destino = -1;
  int maisAntigo = 0;
  for (int i = 0; i < MAX_CANDIDATOS; ++i) {
    const bool ocupado = candidatos[i].vistoEmMs != 0;
    if (ocupado && mesmoMac(candidatos[i].mac, mac)) {
      candidatos[i].vistoEmMs = agora;
      return;
    }
    const bool expirado = !ocupado ||
        (uint32_t)(agora - candidatos[i].vistoEmMs) > TIMEOUT_CANDIDATO_MS;
    if (expirado && destino < 0) destino = i;
    if (candidatos[i].vistoEmMs < candidatos[maisAntigo].vistoEmMs) {
      maisAntigo = i;
    }
  }
  if (destino < 0) destino = maisAntigo;
  memcpy(candidatos[destino].mac, mac, 6);
  candidatos[destino].vistoEmMs = agora;
}

void aoReceber(const esp_now_recv_info_t *info, const uint8_t *dados,
               int tamanho) {
  if (info == nullptr || dados == nullptr ||
      tamanho != static_cast<int>(sizeof(ESPNowLinkPacket))) {
    return;
  }
  ESPNowLinkPacket pacote;
  memcpy(&pacote, dados, sizeof(pacote));
  if (pacote.magic != ESPNOW_LINK_MAGIC ||
      pacote.version != ESPNOW_LINK_VERSION ||
      pacote.checksum != espNowLinkChecksum(pacote)) {
    return;
  }
  const uint8_t *origem = info->src_addr;
  if (mesmoMac(origem, meuMac)) return;
  const uint32_t agora = millis() == 0 ? 1 : millis();

  portENTER_CRITICAL(&trava);
  if (pacote.type == ESPNOW_LINK_ANNOUNCE) {
    registrarCandidato(origem, agora);
  } else if (temParceiro && mesmoMac(origem, macParceiro)) {
    // Somente o MAC escolhido pelo operador pode abrir ou manter o vinculo.
    if (pacote.type == ESPNOW_LINK_PAIR) ackPendente = true;
    if (pacote.type == ESPNOW_LINK_PAIR ||
        pacote.type == ESPNOW_LINK_PAIR_ACK ||
        pacote.type == ESPNOW_LINK_STATE) {
      parceiroConectado = true;
      ultimoPacoteParceiroMs = agora;
      parceiroDefOn = pacote.defOn != 0;
      parceiroPapel = pacote.role <= 2 ? static_cast<Papel>(pacote.role)
                                       : Papel::Nenhum;
    }
  }
  portEXIT_CRITICAL(&trava);
}

void carregarParceiro() {
  Preferences preferencias;
  if (!preferencias.begin("espnow-link", true)) return;
  uint8_t mac[6] = {};
  if (preferencias.getBytes("peer", mac, 6) == 6) {
    memcpy(macParceiro, mac, 6);
    temParceiro = true;
  }
  preferencias.end();
}

bool gravarParceiro(const uint8_t *mac) {
  Preferences preferencias;
  if (!preferencias.begin("espnow-link", false)) return false;
  const bool ok = mac == nullptr ? preferencias.remove("peer")
                                 : preferencias.putBytes("peer", mac, 6) == 6;
  preferencias.end();
  return ok;
}

uint8_t copiarCandidatos(uint8_t macs[][6], uint8_t maximo) {
  const uint32_t agora = millis();
  uint8_t total = 0;
  portENTER_CRITICAL(&trava);
  for (int i = 0; i < MAX_CANDIDATOS && total < maximo; ++i) {
    if (candidatos[i].vistoEmMs != 0 &&
        (uint32_t)(agora - candidatos[i].vistoEmMs) <= TIMEOUT_CANDIDATO_MS) {
      memcpy(macs[total++], candidatos[i].mac, 6);
    }
  }
  portEXIT_CRITICAL(&trava);
  return total;
}

}  // namespace

void begin() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_get_mac(WIFI_IF_STA, meuMac);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ERRO: esp_now_init falhou; vinculo desativado.");
    return;
  }
  esp_now_register_recv_cb(aoReceber);
  adicionarPeer(BROADCAST);
  carregarParceiro();
  if (temParceiro) adicionarPeer(macParceiro);
  iniciado = true;
  Serial.printf("ESP-NOW MAC local: %02X:%02X:%02X:%02X:%02X:%02X\n",
                meuMac[0], meuMac[1], meuMac[2], meuMac[3], meuMac[4],
                meuMac[5]);
}

void update() {
  if (!iniciado) return;
  const uint32_t agora = millis();

  portENTER_CRITICAL(&trava);
  if (parceiroConectado &&
      (uint32_t)(agora - ultimoPacoteParceiroMs) > TIMEOUT_VINCULO_MS) {
    parceiroConectado = false;
    parceiroDefOn = false;
    parceiroPapel = Papel::Nenhum;
  }
  const bool conectadoAgora = parceiroConectado;
  const bool enviarAck = ackPendente;
  ackPendente = false;
  const bool parceiroGravado = temParceiro;
  uint8_t destino[6];
  memcpy(destino, macParceiro, 6);
  portEXIT_CRITICAL(&trava);

  if (!parceiroGravado) {
    uint8_t macs[MAX_CANDIDATOS][6];
    if (buscaAtiva && copiarCandidatos(macs, MAX_CANDIDATOS) == 1) {
      selecionarMac(macs[0]);
    }
  }

  if (agora - ultimoAnuncioMs >= INTERVALO_ANUNCIO_MS) {
    ultimoAnuncioMs = agora;
    enviar(BROADCAST, ESPNOW_LINK_ANNOUNCE);
  }
  if (!parceiroGravado) return;

  if (enviarAck) enviar(destino, ESPNOW_LINK_PAIR_ACK);
  if (!conectadoAgora) {
    if (agora - ultimoPairMs >= INTERVALO_PAIR_MS) {
      ultimoPairMs = agora;
      enviar(destino, ESPNOW_LINK_PAIR);
    }
  } else if (agora - ultimoStateMs >= INTERVALO_STATE_MS) {
    ultimoStateMs = agora;
    enviar(destino, ESPNOW_LINK_STATE);
  }
}

void iniciarBusca() {
  buscaAtiva = true;
}

void definirEstadoLocal(Papel papel, bool defOn) {
  papelLocal = papel;
  defOnLocal = defOn;
}

Estado estado() {
  portENTER_CRITICAL(&trava);
  const bool gravado = temParceiro;
  const bool ligado = parceiroConectado;
  portEXIT_CRITICAL(&trava);
  if (gravado) return ligado ? Estado::Conectado : Estado::Definindo;
  uint8_t macs[MAX_CANDIDATOS][6];
  return copiarCandidatos(macs, MAX_CANDIDATOS) > 1 ? Estado::EscolhaManual
                                                     : Estado::Procurando;
}

void macLocal(uint8_t mac[6]) {
  memcpy(mac, meuMac, 6);
}

uint8_t candidatosOnline(uint8_t macs[][6], uint8_t maximo) {
  return copiarCandidatos(macs, maximo);
}

bool selecionarMac(const uint8_t mac[6]) {
  if (!iniciado || mesmoMac(mac, meuMac) || !adicionarPeer(mac)) return false;
  if (!gravarParceiro(mac)) {
    Serial.println("ERRO: nao foi possivel gravar o MAC do parceiro na NVS.");
  }
  portENTER_CRITICAL(&trava);
  memcpy(macParceiro, mac, 6);
  temParceiro = true;
  parceiroConectado = false;
  parceiroDefOn = false;
  parceiroPapel = Papel::Nenhum;
  portEXIT_CRITICAL(&trava);
  return true;
}

bool possuiVinculo() {
  return temParceiro;
}

void macVinculo(uint8_t mac[6]) {
  portENTER_CRITICAL(&trava);
  memcpy(mac, macParceiro, 6);
  portEXIT_CRITICAL(&trava);
}

void esquecerVinculo() {
  buscaAtiva = false;
  if (!temParceiro) return;
  uint8_t antigo[6];
  portENTER_CRITICAL(&trava);
  memcpy(antigo, macParceiro, 6);
  temParceiro = false;
  parceiroConectado = false;
  parceiroDefOn = false;
  parceiroPapel = Papel::Nenhum;
  portEXIT_CRITICAL(&trava);
  esp_now_del_peer(antigo);
  gravarParceiro(nullptr);
}

bool conectado() {
  portENTER_CRITICAL(&trava);
  const bool ligado = temParceiro && parceiroConectado;
  portEXIT_CRITICAL(&trava);
  return ligado;
}

bool defesaParceiroDesligada() {
  portENTER_CRITICAL(&trava);
  const bool resultado = temParceiro && parceiroConectado && !parceiroDefOn &&
                         parceiroPapel != Papel::Ataque;
  portEXIT_CRITICAL(&trava);
  return resultado;
}

}  // namespace ESPNowLink
