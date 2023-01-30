#include "ring_buf.h"

r_size_t ring_get_free(t_ring *ring)
{
  return RING_BUF_SIZE - ring->w_pos + ring->r_pos;
}

r_data_t* ring_put_byte(t_ring *ring, r_data_t data) {
  if (ring->w_pos - ring->r_pos == RING_BUF_SIZE) return 0;
  r_data_t *ret = &ring->data[ring->w_pos++ % RING_BUF_SIZE];
  *ret = data;
  return ret;
}

r_data_t* ring_get_byte(t_ring *ring) {
  if (ring->w_pos == ring->r_pos) return 0;
  return &ring->data[ring->r_pos++ % RING_BUF_SIZE];
}

r_size_t ring_put_data(t_ring *ring, r_data_t *data, r_size_t size) {
  for (int i = 0; i < size; i++)
    if (ring_put_byte(ring, data[i]) == 0) return i;
  return size;
}

r_size_t ring_get_data(t_ring *ring, r_data_t *data, r_size_t size) {
  r_data_t *data_byte;
  for (int i = 0; i < size; i++) {
    data_byte = ring_get_byte(ring);
    if (data_byte == 0) return i;
    data[i] = *data_byte;
  }
  return size;
}

void ring_flush(t_ring *ring)
{
	ring->w_pos = ring->r_pos;
}
