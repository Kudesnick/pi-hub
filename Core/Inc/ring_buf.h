#include <stdint.h>

#pragma once

#define RING_BUF_SIZE 64

typedef uint8_t r_data_t;
typedef uint16_t r_size_t;

typedef struct {
  r_size_t w_pos;
  r_size_t r_pos;
  r_data_t data[RING_BUF_SIZE];
} t_ring;

r_size_t ring_get_free(t_ring *ring);
r_data_t* ring_put_byte(t_ring *ring, r_data_t data);
r_data_t* ring_get_byte(t_ring *ring);
r_size_t ring_put_data(t_ring *ring, r_data_t *data, r_size_t size);
r_size_t ring_get_data(t_ring *ring, r_data_t *data, r_size_t size);
void ring_flush(t_ring *ring);
