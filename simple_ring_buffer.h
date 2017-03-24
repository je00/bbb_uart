#ifndef __SIMPLE_RING_BUFFER_H__
#define __SIMPLE_RING_BUFFER_H__

#include <asm/string.h>
#include <linux/kernel.h>
#include <linux/slab.h>

struct simple_ring_buffer {
	uint8_t *buf;
	int size;
	int head;
	int tail;
	int cur_size;
};

int simple_ring_buffer_init(struct simple_ring_buffer* ring, int size)
{
    ring->buf = (uint8_t*)kmalloc(size, GFP_KERNEL);
    if (!ring->buf)
        return -ENOMEM;
	ring->cur_size = 0;
    ring->size = size;
    ring->head = 0;
	ring->tail = 0;
    return 0;
}

void simple_ring_buffer_destroy(struct simple_ring_buffer *ring)
{
    ring->size = 0;
    ring->head = 0;
	ring->tail = 0;
    kfree(ring->buf);
}

void simple_ring_buffer_push_back(struct simple_ring_buffer *ring, uint8_t d)
{
    ring->buf[ring->tail] = d;

	ring->tail++;
	if (ring->tail >= ring->size) {
        ring->tail = 0;
	}
	
	ring->cur_size++;
	if (ring->cur_size > ring->size)
		ring->cur_size = ring->size;
	
	if (ring->cur_size == ring->size) {
		ring->head++;
		if (ring->head == ring->size) {
			ring->head = 0;
		}
	}
}

void simple_ring_buffer_npush_back(struct simple_ring_buffer *ring, uint8_t *buf, int size)
{
	int i = 0;
	for (; i < size; i++) {
		simple_ring_buffer_push_back(ring, buf[i]);
	}
}

uint8_t simple_ring_buffer_get(struct simple_ring_buffer *ring)
{
	char c = ring->buf[ring->head];
	ring->cur_size--;
	
	ring->head++;
	if (ring->head == ring->size)
		ring->head = 0;
	
    return c;
}

int simple_ring_buffer_get_n(struct simple_ring_buffer *ring, uint8_t *buf, int size)
{	
	int i;
	if (size > ring->cur_size)
		size = ring->cur_size;
	for (i = 0; i < size; i++) {
		buf[i] = simple_ring_buffer_get(ring);
	}
	return size;
}

int simple_ring_buffer_is_empty(struct simple_ring_buffer *ring)
{
	if (ring->cur_size <= 0)
		return 1;
	return 0;
}

#endif
