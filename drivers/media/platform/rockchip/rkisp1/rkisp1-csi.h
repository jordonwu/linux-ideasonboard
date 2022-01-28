#ifndef _RKISP1_CSI_H
#define _RKISP1_CSI_H

#include "rkisp1-common.h"

int rkisp1_csi_link_sensor(struct rkisp1_device *rkisp1,
			   struct v4l2_subdev *sd,
			   struct rkisp1_sensor_async *s_asd);

#endif /* _RKISP1_CSI_H */
