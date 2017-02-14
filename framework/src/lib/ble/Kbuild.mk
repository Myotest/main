obj-y += bas/
obj-y += dis/
obj-y += gap/
obj-y += hrs/
obj-y += lns/
obj-y += rscs/
obj-y += acc/
obj-y += tcmd/
obj-$(CONFIG_BLE_APP) += ble_app.o

# for the nordic rpc functions
CFLAGS_ble_app.o = -I$(T)/external/zephyr/drivers/nble_curie/
