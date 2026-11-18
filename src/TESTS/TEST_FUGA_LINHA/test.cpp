#include <Arduino.h>
#include <FugaLinha.h>
#include <assert.h>
#include <math.h>
#include <stdint.h>

TestSerial Serial;

int main() {
  LDRFixed ldr;
  FugaLinha fuga(ldr, 0.7f);
  ldr.regioes[LDRFixed::Superior] = true;
  assert(fuga.atualizar(true, 0));
  assert(fuga.origemCorrecao() == FugaLinha::OrigemCorrecao::ParadaFrontal);
  assert(fabsf(fuga.vetorX()) < 0.001f);
  assert(fabsf(fuga.vetorY()) < 0.001f);
  assert(fuga.atualizar(true, 49));
  assert(fuga.origemCorrecao() == FugaLinha::OrigemCorrecao::ParadaFrontal);
  ldr.regioes[LDRFixed::Superior] = false;
  assert(fuga.atualizar(true, 50));
  assert(fuga.origemCorrecao() == FugaLinha::OrigemCorrecao::RetornoFrontal);
  assert(fabsf(fuga.vetorX()) < 0.001f);
  assert(fabsf(fuga.vetorY() + 0.7f) < 0.001f);
  assert(fuga.atualizar(true, 299));
  assert(fuga.origemCorrecao() == FugaLinha::OrigemCorrecao::RetornoFrontal);
  assert(!fuga.atualizar(true, 300));

  fuga.resetar();
  ldr.regioes[LDRFixed::Superior] = true;
  assert(fuga.atualizar(true, 1000));
  assert(fuga.atualizar(true, 1299));
  ldr.regioes[LDRFixed::Superior] = false;
  assert(fuga.atualizar(true, 1300));
  assert(fuga.atualizar(true, 1419));
  assert(!fuga.atualizar(true, 1420));

  const LDRFixed::Regiao outrasRegioes[] = {
      LDRFixed::Esquerda, LDRFixed::Direita, LDRFixed::Inferior,
      LDRFixed::Centro};
  for (const LDRFixed::Regiao regiao : outrasRegioes) {
    fuga.resetar();
    ldr.regioes[regiao] = true;
    assert(fuga.atualizar(true, 2000));
    ldr.regioes[regiao] = false;
    assert(fuga.atualizar(true, 2100));
    assert(fuga.atualizar(true, 2499));
    assert(!fuga.atualizar(true, 2500));
  }

  fuga.resetar();
  ldr.regioes[LDRFixed::Superior] = true;
  assert(fuga.atualizar(true, 3000));
  assert(fuga.atualizar(true, 3050));
  assert(fuga.vetorY() < 0);
  assert(fuga.atualizar(true, 3300));
  assert(fuga.origemCorrecao() == FugaLinha::OrigemCorrecao::Direta);
  assert(fuga.atualizar(true, 3750));
  assert(fuga.vetorY() < 0);
  assert(fuga.atualizar(true, 3999));
  assert(!fuga.atualizar(true, 4000));
  assert(fuga.expirada());
  ldr.regioes[LDRFixed::Superior] = false;
  assert(!fuga.atualizar(true, 4010));
  assert(!fuga.atualizar(true, 4130));
  assert(!fuga.expirada());

  fuga.resetar();
  ldr.regioes[LDRFixed::Superior] = true;
  assert(fuga.atualizar(true, 5000));
  ldr.regioes[LDRFixed::Superior] = false;
  assert(fuga.atualizar(true, 5050));
  ldr.regioes[LDRFixed::Inferior] = true;
  assert(fuga.atualizar(true, 5100));
  assert(fuga.vetorY() > 0);
  ldr.regioes[LDRFixed::Inferior] = false;
  assert(fuga.atualizar(true, 5150));
  assert(fuga.atualizar(true, 5549));
  assert(!fuga.atualizar(true, 5550));

  fuga.resetar();
  ldr.regioes[LDRFixed::Superior] = true;
  assert(fuga.atualizar(true, UINT32_MAX - 100));
  ldr.regioes[LDRFixed::Superior] = false;
  assert(fuga.atualizar(true, UINT32_MAX - 51));
  assert(fuga.origemCorrecao() == FugaLinha::OrigemCorrecao::ParadaFrontal);
  assert(fuga.atualizar(true, UINT32_MAX - 50));
  assert(fuga.origemCorrecao() == FugaLinha::OrigemCorrecao::RetornoFrontal);
  assert(fuga.atualizar(true, 198));
  assert(!fuga.atualizar(true, 199));

  fuga.resetar();
  ldr.regioes[LDRFixed::Esquerda] = true;
  assert(fuga.atualizar(true, 7000));
  ldr.regioes[LDRFixed::Superior] = true;
  assert(fuga.atualizar(true, 7100));
  assert(fuga.origemCorrecao() == FugaLinha::OrigemCorrecao::ParadaFrontal);
  ldr.regioes[LDRFixed::Esquerda] = false;
  ldr.regioes[LDRFixed::Superior] = false;
  assert(fuga.atualizar(true, 7150));
  assert(fuga.origemCorrecao() == FugaLinha::OrigemCorrecao::RetornoFrontal);
  assert(fuga.atualizar(true, 7399));
  assert(!fuga.atualizar(true, 7400));
}
