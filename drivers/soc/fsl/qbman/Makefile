# Common
obj-$(CONFIG_FSL_DPA)				+= dpaa_resource.o

# BMan
obj-$(CONFIG_FSL_BMAN)                          += bman.o bman_utils.o
obj-$(CONFIG_FSL_BMAN_PORTAL)                   += bman_portal.o bman_utils.o bman_api.o
obj-$(CONFIG_FSL_BMAN_DEBUGFS)                  += bman_debugfs.o
obj-$(CONFIG_FSL_BMAN_TEST)                     += bman-test.o
bman-test-y                                      = bman_test.o
bman-test-$(CONFIG_FSL_BMAN_TEST_API)           += bman_test_api.o
bman-test-$(CONFIG_FSL_BMAN_TEST_THRESH)        += bman_test_thresh.o

# QMan
obj-$(CONFIG_FSL_QMAN)				+= qman_api.o qman_utils.o qman_driver.o
obj-$(CONFIG_FSL_QMAN_CONFIG)			+= qman.o qman_portal.o

