#include <FugaLinha.h>
#include <Arduino.h>
#include <math.h>

namespace {
constexpr uint32_t TEMPO_MINIMO_FUGA_MS = 300;
constexpr uint32_t TEMPO_LINHA_LIMPA_MS = 120;
constexpr uint32_t TEMPO_RETORNO_APOS_LINHA_MS = 400;
constexpr uint32_t TEMPO_PARADA_FRONTAL_MS = 50;
constexpr uint32_t TEMPO_RETORNO_FRONTAL_MS = 250;
constexpr uint32_t TEMPO_MAXIMO_FUGA_MS = 1000;
constexpr uint32_t JANELA_MEMORIA_PERIFERICA_MS = 3000;
}

FugaLinha::FugaLinha(LDRFixed &ldr, float velocidade)
    : _ldr(ldr), _velocidade(velocidade) {}

bool FugaLinha::atualizar(bool habilitado, uint32_t agoraMs) {
  float x = 0.0f;
  float y = 0.0f;
  float periferiaX = 0.0f;
  float periferiaY = 0.0f;
  const bool centroDetectado =
      habilitado && _ldr.regiaoBranca(LDRFixed::Centro);
  const bool esquerdaDetectada =
      habilitado && _ldr.regiaoBranca(LDRFixed::Esquerda);
  const bool direitaDetectada =
      habilitado && _ldr.regiaoBranca(LDRFixed::Direita);
  const bool inferiorDetectado =
      habilitado && _ldr.regiaoBranca(LDRFixed::Inferior);
  const bool superiorDetectado =
      habilitado && _ldr.regiaoBranca(LDRFixed::Superior);
  const bool novaDeteccaoSuperior = superiorDetectado && !_superiorAnterior;
  _superiorAnterior = superiorDetectado;
  const bool periferiaDetectada = esquerdaDetectada || direitaDetectada ||
                                  inferiorDetectado || superiorDetectado;

  if (esquerdaDetectada != direitaDetectada) {
    if (esquerdaDetectada) {
      x -= _velocidade;
      periferiaX -= _velocidade;
    } else {
      x += _velocidade;
      periferiaX += _velocidade;
    }
  }
  // Frente recua e traseira avanca; as duas juntas se anulam e deixam a decisao
  // para a memoria/fallback. O centro isolado nao tem direcao propria.
  if (superiorDetectado != inferiorDetectado) {
    const float sentido = inferiorDetectado ? 1.0f : -1.0f;
    y += sentido * _velocidade;
    periferiaY += sentido * _velocidade;
  } else if (!superiorDetectado && centroDetectado) {
    y -= _velocidade;
  }

  const bool linhaDetectada = centroDetectado || periferiaDetectada;
  if (linhaDetectada) {
    _origemCorrecao = OrigemCorrecao::Direta;
  } else if (!_ativa) {
    _origemCorrecao = OrigemCorrecao::Nenhuma;
  }

  if (_expirada) {
    _origemCorrecao = OrigemCorrecao::Nenhuma;
    if (linhaDetectada) {
      _temporizadorLinhaLimpaAtivo = false;
      return false;
    }

    if (!_temporizadorLinhaLimpaAtivo) {
      _inicioLinhaLimpaMs = agoraMs;
      _temporizadorLinhaLimpaAtivo = true;
    }
    if (static_cast<uint32_t>(agoraMs - _inicioLinhaLimpaMs) >=
        TEMPO_LINHA_LIMPA_MS) {
      _expirada = false;
      _ativa = false;
      _temporizadorLinhaLimpaAtivo = false;
      _correcaoPerifericaValida = false;
      Serial.println(
          "Fuga LDR rearmada; linha permaneceu limpa apos parada de seguranca.");
    }
    return false;
  }

  if (periferiaDetectada &&
      (periferiaX != 0.0f || periferiaY != 0.0f)) {
    _ultimaCorrecaoPerifericaX = periferiaX;
    _ultimaCorrecaoPerifericaY = periferiaY;
    _ultimaDeteccaoPerifericaMs = agoraMs;
    _correcaoPerifericaValida = true;
  }

  if (centroDetectado && _correcaoPerifericaValida &&
      static_cast<uint32_t>(agoraMs - _ultimaDeteccaoPerifericaMs) <=
          JANELA_MEMORIA_PERIFERICA_MS) {
    x = _ultimaCorrecaoPerifericaX;
    y = _ultimaCorrecaoPerifericaY;
    // Com periferia ativa agora, a memoria acabou de ser atualizada e coincide
    // com a leitura direta; so e "memoria" quando a periferia esta em conflito.
    if (periferiaX == 0.0f && periferiaY == 0.0f) {
      _origemCorrecao = OrigemCorrecao::MemoriaPeriferica;
    }
  }

  float magnitude = sqrtf(x * x + y * y);
  if (linhaDetectada && magnitude <= 0.001f) {
    if (_correcaoPerifericaValida &&
        static_cast<uint32_t>(agoraMs - _ultimaDeteccaoPerifericaMs) <=
            JANELA_MEMORIA_PERIFERICA_MS) {
      x = _ultimaCorrecaoPerifericaX;
      y = _ultimaCorrecaoPerifericaY;
      _origemCorrecao = OrigemCorrecao::MemoriaPeriferica;
    } else {
      x = 0.0f;
      y = -_velocidade;
      _origemCorrecao = OrigemCorrecao::FallbackCentro;
    }
    magnitude = sqrtf(x * x + y * y);
  }

  if (magnitude > _velocidade && magnitude > 0.0f) {
    const float escala = _velocidade / magnitude;
    x *= escala;
    y *= escala;
  }

  if (linhaDetectada) {
    if (!_ativa) {
      _ativa = true;
      _inicioFugaMs = agoraMs;
      Serial.println(
          "Fuga LDR ativada; aguardando sensores indicarem linha limpa.");
    }
    if (!_temporizadorDeteccaoAtivo) {
      _inicioDeteccaoMs = agoraMs;
      _temporizadorDeteccaoAtivo = true;
    }
    if (static_cast<uint32_t>(agoraMs - _inicioDeteccaoMs) >=
        TEMPO_MAXIMO_FUGA_MS) {
      _ativa = false;
      _expirada = true;
      _temporizadorLinhaLimpaAtivo = false;
      _temporizadorDeteccaoAtivo = false;
      _retornoFrontal = false;
      Serial.println(
          "Fuga LDR excedeu 1 s; motores devem parar ate a linha ficar limpa.");
      return false;
    }
    if (inferiorDetectado) {
      _retornoFrontal = false;
    } else if (novaDeteccaoSuperior) {
      _retornoFrontal = true;
      _inicioFrontalMs = agoraMs;
      Serial.println("Fuga frontal: parada de 50 ms, seguida de recuo.");
    }
    if (_retornoFrontal &&
        static_cast<uint32_t>(agoraMs - _inicioFrontalMs) <
            TEMPO_PARADA_FRONTAL_MS + TEMPO_RETORNO_FRONTAL_MS) {
      const bool parado = static_cast<uint32_t>(agoraMs - _inicioFrontalMs) <
                          TEMPO_PARADA_FRONTAL_MS;
      _vetorX = 0.0f;
      _vetorY = parado ? 0.0f : -_velocidade;
      _origemCorrecao = parado ? OrigemCorrecao::ParadaFrontal
                               : OrigemCorrecao::RetornoFrontal;
      _temporizadorLinhaLimpaAtivo = false;
      return true;
    }
    _vetorX = x;
    _vetorY = y;
    _temporizadorLinhaLimpaAtivo = false;
    return true;
  }

  _temporizadorDeteccaoAtivo = false;
  if (!_ativa) return false;

  if (!_temporizadorLinhaLimpaAtivo) {
    _inicioLinhaLimpaMs = agoraMs;
    _temporizadorLinhaLimpaAtivo = true;
  }

  const bool tempoMinimoCumprido =
      static_cast<uint32_t>(agoraMs - _inicioFugaMs) >=
      TEMPO_MINIMO_FUGA_MS;
  const bool frontalConcluida =
      !_retornoFrontal ||
      static_cast<uint32_t>(agoraMs - _inicioFrontalMs) >=
          TEMPO_PARADA_FRONTAL_MS + TEMPO_RETORNO_FRONTAL_MS;
  const bool linhaLimpaEstavel =
      static_cast<uint32_t>(agoraMs - _inicioLinhaLimpaMs) >=
      (_retornoFrontal ? TEMPO_LINHA_LIMPA_MS
                       : TEMPO_RETORNO_APOS_LINHA_MS);
  if (tempoMinimoCumprido && frontalConcluida && linhaLimpaEstavel) {
    _ativa = false;
    _temporizadorLinhaLimpaAtivo = false;
    _retornoFrontal = false;
    _origemCorrecao = OrigemCorrecao::Nenhuma;
    Serial.println("Fuga LDR encerrada; retomando perseguicao.");
    return false;
  }

  if (_retornoFrontal) {
    const bool parado = static_cast<uint32_t>(agoraMs - _inicioFrontalMs) <
                        TEMPO_PARADA_FRONTAL_MS;
    _vetorX = 0.0f;
    _vetorY = parado ? 0.0f : -_velocidade;
    _origemCorrecao = parado ? OrigemCorrecao::ParadaFrontal
                             : OrigemCorrecao::RetornoFrontal;
  }
  return true;
}

void FugaLinha::resetar() {
  _ativa = false;
  _expirada = false;
  _temporizadorLinhaLimpaAtivo = false;
  _correcaoPerifericaValida = false;
  _retornoFrontal = false;
  _superiorAnterior = false;
  _temporizadorDeteccaoAtivo = false;
  _origemCorrecao = OrigemCorrecao::Nenhuma;
}

bool FugaLinha::expirada() const { return _expirada; }

float FugaLinha::vetorX() const { return _vetorX; }

float FugaLinha::vetorY() const { return _vetorY; }

bool FugaLinha::regiaoBranca(LDRFixed::Regiao regiao) const {
  return _ldr.regiaoBranca(regiao);
}

FugaLinha::OrigemCorrecao FugaLinha::origemCorrecao() const {
  return _origemCorrecao;
}
