/**
 * Copyright 2020 Mi
 */

#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/jiffies.h>
#include <sound/asound.h>
#include <sound/soc.h>
#include <sound/control.h>

struct model_info {
	void *data;
	uint32_t len;
};

struct voice_sound_model {
	dma_addr_t  phys;
	void  *data;
	size_t  size; // USELESS
	struct dma_buf  *dma_buf;
	uint32_t  mem_map_handle;
};

unsigned int send_data_add_component_controls(void *component);

