#include "ring_buf.h"

r_size_t ring_get_free(t_ring *ring)
{
  return RING_BUF_SIZE - ring->w_pos + ring->r_pos;
}

bool ring_put_byte(t_ring *ring, r_data_t data) {
  if (ring->w_pos - ring->r_pos == RING_BUF_SIZE) return false;
  ring->data[ring->w_pos % RING_BUF_SIZE] = data;
  ring->w_pos++;
  return true;
}

bool ring_get_byte(t_ring *ring, r_data_t *_data) {
  if (ring->w_pos == ring->r_pos) return false;
  *_data = ring->data[ring->r_pos % RING_BUF_SIZE];
  ring->r_pos++;
  return true;
}

r_size_t ring_put_data(t_ring *ring, r_data_t *data, r_size_t size) {
  for (int i = 0; i < size; i++)
    if (ring_put_byte(ring, data[i]) == 0) return i;
  return size;
}

r_size_t ring_get_data(t_ring *ring, r_data_t *data, r_size_t size) {
  for (int i = 0; i < size; i++)
	if (!ring_get_byte(ring, &data[i])) return i;
  return size;
}

void ring_flush(t_ring *ring)
{
	ring->w_pos = ring->r_pos;
}
