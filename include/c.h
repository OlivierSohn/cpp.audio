#ifndef IMJ_AUDIO_C_H
#define IMJ_AUDIO_C_H


typedef struct {

  // Expressed in 'radians / pi',
  // i.e [-1,1] covers the whole trigonometric circle
  float phase;

  // Between 0 (included) and 1 (included).
  float volume;
} harmonicProperties_t;

#endif
