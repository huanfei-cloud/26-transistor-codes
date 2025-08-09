#ifndef TASK_COMMUNICATEWITHPC_H
#define TASK_COMMUNICATEWITHPC_H

#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "Protocol_UpperComputer.h"
#include "Extern_Handles.h"
#include "usbd_cdc_if.h"

void USBCommunicateTask(void const *argument);

#endif
