ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
dtbo-$(CONFIG_ARCH_SDM845) += \
	sdm845-v2-htc_ime-xb-overlay.dtbo \
	sdm845-v2-htc_ime-xc-overlay.dtbo \
	sdm845-v2.1-htc_ime-xc-overlay.dtbo \
	sdm845-v2.1-htc_ime-xd-overlay.dtbo \
	sdm845-v2.1-htc_ime-a-overlay.dtbo \
	sdm845-v2-htc_imeuhljpn-xb-overlay.dtbo \
	sdm845-v2.1-htc_imeuhljpn-xb-overlay.dtbo \
	sdm845-v2.1-htc_imeuhljpn-xc-overlay.dtbo \
	sdm845-v2.1-htc_imeuhljpn-a-overlay.dtbo \
	sdm845-v2.1-htc_imewhltmus-xa-overlay.dtbo \
	sdm845-v2.1-htc_imewhltmus-xb-overlay.dtbo

$(obj)/%.dtb_qcom-clone: $(src)/../qcom/%.dts FORCE
	$(call if_changed_dep,dtc)

#sdm845-htc_sdm845-evm-overlay.dtbo-base := sdm845.dtb_qcom-clone
#sdm845-htc_ime-xa-overlay.dtbo-base := sdm845.dtb_qcom-clone
sdm845-v2-htc_ime-xb-overlay.dtbo-base := sdm845-v2.dtb_qcom-clone
sdm845-v2-htc_ime-xc-overlay.dtbo-base := sdm845-v2.dtb_qcom-clone
sdm845-v2.1-htc_ime-xc-overlay.dtbo-base := sdm845-v2.1.dtb_qcom-clone
sdm845-v2.1-htc_ime-xd-overlay.dtbo-base := sdm845-v2.1.dtb_qcom-clone
sdm845-v2.1-htc_ime-a-overlay.dtbo-base := sdm845-v2.1.dtb_qcom-clone
#sdm845-htc_imeuhljpn-xa-overlay.dtbo-base := sdm845.dtb_qcom-clone
sdm845-v2-htc_imeuhljpn-xb-overlay.dtbo-base := sdm845-v2.dtb_qcom-clone
sdm845-v2.1-htc_imeuhljpn-xb-overlay.dtbo-base := sdm845-v2.1.dtb_qcom-clone
sdm845-v2.1-htc_imeuhljpn-xc-overlay.dtbo-base := sdm845-v2.1.dtb_qcom-clone
sdm845-v2.1-htc_imeuhljpn-a-overlay.dtbo-base := sdm845-v2.1.dtb_qcom-clone
sdm845-v2.1-htc_imewhltmus-xa-overlay.dtbo-base := sdm845-v2.1.dtb_qcom-clone
sdm845-v2.1-htc_imewhltmus-xb-overlay.dtbo-base := sdm845-v2.1.dtb_qcom-clone
else
dtb-$(CONFIG_ARCH_SDM845) += \
	sdm845-htc_sdm845-evm.dtb
endif

always         := $(dtb-y)
subdir-y       := $(dts-dirs)
clean-files    := *.dtb
