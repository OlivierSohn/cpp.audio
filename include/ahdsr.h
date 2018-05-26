#pragma once

struct AHDSR_s {
  int attack;
  int hold;
  int decay;
  int release;
  float sustain;
};

typedef struct AHDSR_s AHDSR_t;
