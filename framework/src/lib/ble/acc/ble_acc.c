/*
  * Copyright (c) 2015-2016, Intel Corporation. All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted provided that the following conditions are met:
  *
  * 1. Redistributions of source code must retain the above copyright notice,
  * this list of conditions and the following disclaimer.
  *
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  * this list of conditions and the following disclaimer in the documentation
  * and/or other materials provided with the distribution.
  *
  * 3. Neither the name of the copyright holder nor the names of its contributors
  * may be used to endorse or promote products derived from this software without
  * specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
  * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  * POSSIBILITY OF SUCH DAMAGE.
  */

 #include "lib/ble/acc/ble_acc.h"
 #include "infra/log.h"

 #include <bluetooth/gatt.h>
 #include <bluetooth/uuid.h>
 #include "../../internal/projects/curie_streaming/quark/rawdata.h"


/* Main UUID */
 static const struct bt_uuid_128 Acc_Main_uuid = {
 	.uuid = { .type = BT_UUID_TYPE_128},
 	.val = {  0x71, 0x26, 0x00, 0x1f, 0x36, 0x92, 0x06, 0xb0,
            0xe6, 0x11, 0x12, 0xed, 0x00, 0xBB, 0x46, 0xbf },
 };

 /* Data UUID */
 static const struct bt_uuid_128 Acc_Data_uuid = {
 	.uuid = { .type = BT_UUID_TYPE_128},
 	.val = {  0x71, 0x26, 0x00, 0x1f, 0x36, 0x92, 0x06, 0xb0,
            0xe6, 0x11, 0x12, 0xed, 0x01, 0xBB, 0x46, 0xbf
          },
 };

 /* Config UUID */
 static const struct bt_uuid_128 Acc_Config_uuid = {
 	.uuid = { .type = BT_UUID_TYPE_128},
 	.val = {  0x71, 0x26, 0x00, 0x1f, 0x36, 0x92, 0x06, 0xb0,
            0xe6, 0x11, 0x12, 0xed, 0x02, 0xBB, 0x46, 0xbf
          },
 };


/* Acc enable call back */
/*
static ssize_t write_acc_enable(struct bt_conn *conn,
				   const struct bt_gatt_attr *attr,
				   const void *buf, uint16_t len,
				   uint16_t offset)
{
  const uint8_t *data = buf;

  if(len > 1)
    pr_error(LOG_MODULE_BLE, "Wrong acc sampling enable value should be 0 or 1");
  else
  {
    if(*data == 1)
    {
      pr_info(LOG_MODULE_BLE, "Enable");
      startAcc();
    }
    else
    {
      pr_info(LOG_MODULE_BLE, "Disable");
      stopAcc();
    }

  }
  return len;
}
*/

/* Config write call back */
static ssize_t write_Config(struct bt_conn *conn,
				   const struct bt_gatt_attr *attr,
				   const void *buf, uint16_t len,
				   uint16_t offset)
{
  sconfigAcc config;
  config.freq = *(uint8_t*)buf;

  pr_info(LOG_MODULE_BLE, "len :%d, size conf:%d",len,sizeof(sconfigAcc));

  if(len == sizeof(sconfigAcc))
  {

    if((config.freq & ACC_ENABLE_MASK) == ACC_ENABLE_MASK)
    {
      config.freq = config.freq&(~ACC_ENABLE_MASK);
      pr_info(LOG_MODULE_BLE, "Send config freq:%d",config.freq);
      configAccService(&config);
    }
  }
  return len;
}

 /* Battery Service Variables */

 static struct bt_gatt_ccc_cfg blvl_ccc_cfg[1] = {};

 #define blvl_ccc_cfg_changed NULL


 /* Battery Service Declaration */
 static const struct bt_gatt_attr acc_attrs[] = {
 	BT_GATT_PRIMARY_SERVICE((void*)&Acc_Main_uuid),
 	BT_GATT_CHARACTERISTIC(&Acc_Data_uuid.uuid,BT_GATT_CHRC_NOTIFY),
 	BT_GATT_DESCRIPTOR(&Acc_Data_uuid.uuid, 0 , NULL , NULL/*write_acc_enable*/, NULL),
 	BT_GATT_CCC(blvl_ccc_cfg, blvl_ccc_cfg_changed),

  BT_GATT_CHARACTERISTIC(&Acc_Config_uuid.uuid,BT_GATT_CHRC_WRITE),
 	BT_GATT_DESCRIPTOR(&Acc_Config_uuid.uuid, BT_GATT_PERM_WRITE, NULL,write_Config, NULL),
 	BT_GATT_CCC(blvl_ccc_cfg, blvl_ccc_cfg_changed),

 };

 /* Pointer to the HRS value attribute in the above table */
 static struct bt_gatt_attr const *const acc_value = &acc_attrs[2];

 int ble_acc_init(void)
 {
 	/* cast to discard the const qualifier */
 	return bt_gatt_register((struct bt_gatt_attr *)acc_attrs,
 				ARRAY_SIZE(acc_attrs));
 }

 int ble_acc_update(uint8_t *Data_To_Send, uint8_t len)
 {
 	
 	return bt_gatt_notify(NULL, acc_value, Data_To_Send, len, NULL);
 }

 const struct bt_gatt_attr *ble_acc_attr(void)
 {
 	return acc_value;
 }
