# Overlays for the Raspberry Pi platform

dtb-$(CONFIG_ARCH_LAN969X) += 4_port_switch_carrier_v01.dtbo

targets += dtbs dtbs_install
targets += $(dtbo-y)

always-y	:= $(dtbo-y)
clean-files	:= *.dtbo
