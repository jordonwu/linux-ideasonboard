#ifndef _RKISP1_CSI_H
#define _RKISP1_CSI_H

#include "rkisp1-common.h"

int rkisp1_config_mipi(struct rkisp1_device *rkisp1);

void rkisp1_mipi_start(struct rkisp1_device *rkisp1);
void rkisp1_mipi_stop(struct rkisp1_device *rkisp1);

#endif /* _RKISP1_CSI_H */
