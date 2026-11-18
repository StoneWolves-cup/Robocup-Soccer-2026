#ifndef FUGA_LINHA_H
#define FUGA_LINHA_H

#include <stdint.h>
#include <LDRFixed.h>

class FugaLinha {
public:
  enum class OrigemCorrecao : uint8_t {
    Nenhuma,
    Direta,
    MemoriaPeriferica,
    FallbackCentro,
    ParadaFrontal,
    RetornoFrontal
  };

  FugaLinha(LDRFixed &ldr, float velocidade);

  bool atualizar(bool habilitado, uint32_t agoraMs);
  void resetar();
  bool expirada() const;
  float vetorX() const;
  float vetorY() const;
  bool regiaoBranca(LDRFixed::Regiao regiao) const;
  OrigemCorrecao origemCorrecao() const;

private:
  LDRFixed &_ldr;
  float _velocidade;
  bool _ativa = false;
  bool _expirada = false;
  bool _temporizadorLinhaLimpaAtivo = false;
  bool _correcaoPerifericaValida = false;
  bool _retornoFrontal = false;
  bool _superiorAnterior = false;
  bool _temporizadorDeteccaoAtivo = false;
  OrigemCorrecao _origemCorrecao = OrigemCorrecao::Nenhuma;
  uint32_t _inicioFugaMs = 0;
  uint32_t _inicioDeteccaoMs = 0;
  uint32_t _inicioFrontalMs = 0;
  uint32_t _inicioLinhaLimpaMs = 0;
  uint32_t _ultimaDeteccaoPerifericaMs = 0;
  float _vetorX = 0.0f;
  float _vetorY = 0.0f;
  float _ultimaCorrecaoPerifericaX = 0.0f;
  float _ultimaCorrecaoPerifericaY = 0.0f;
};

#endif
