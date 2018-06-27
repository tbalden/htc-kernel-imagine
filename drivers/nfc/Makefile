#
# Makefile for nfc devices
#

#NFC ++
ifeq ($(findstring JAPAN,$(PRIVATE_SKU_NAME)),JAPAN)
obj-$(CONFIG_SENSORS_NFC_CXD224X) += cxd225x-i2c.o
else
obj-$(CONFIG_SENSORS_NFC_PN553) += nq-nci-htc.o
obj-$(CONFIG_SENSORS_NFC_PN553) += nq-nci-htc-mfg.o
endif
#NFC --
obj-$(CONFIG_NFC_FDP)		+= fdp/
obj-$(CONFIG_NFC_PN544)		+= pn544/
obj-$(CONFIG_NFC_MICROREAD)	+= microread/
obj-$(CONFIG_NFC_PN533)		+= pn533/
obj-$(CONFIG_NFC_WILINK)	+= nfcwilink.o
obj-$(CONFIG_NFC_MEI_PHY)	+= mei_phy.o
obj-$(CONFIG_NFC_SIM)		+= nfcsim.o
obj-$(CONFIG_NFC_PORT100)	+= port100.o
obj-$(CONFIG_NFC_MRVL)		+= nfcmrvl/
obj-$(CONFIG_NFC_TRF7970A)	+= trf7970a.o
obj-$(CONFIG_NFC_ST21NFCA)  	+= st21nfca/
obj-$(CONFIG_NFC_ST_NCI)	+= st-nci/
obj-$(CONFIG_NFC_NXP_NCI)	+= nxp-nci/
obj-$(CONFIG_NFC_S3FWRN5)	+= s3fwrn5/
obj-$(CONFIG_NFC_ST95HF)	+= st95hf/
obj-$(CONFIG_NFC_NQ)		+= nq-nci.o
