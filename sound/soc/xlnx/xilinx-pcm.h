#ifndef __XILINX_PCM_H__
#define __XILINX_PCM_H__

struct device;

int xlnx_pcm_register(struct device *dev);
void xlnx_pcm_unregister(struct device *dev);

#endif
