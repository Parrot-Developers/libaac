
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := libaac
LOCAL_CATEGORY_PATH := libs
LOCAL_DESCRIPTION := AAC bitstream reader/writer library
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include
LOCAL_CFLAGS := -DAAC_API_EXPORTS -fvisibility=hidden -std=gnu99 -D_GNU_SOURCE
LOCAL_SRC_FILES := \
	src/aac_bitstream.c \
	src/aac_ctx.c \
	src/aac_dump.c \
	src/aac_reader.c \
	src/aac_types.c \
	src/aac_writer.c \
	src/aac.c

LOCAL_PRIVATE_LIBRARIES := \
	json \
	libaudio-defs \
	libulog

ifeq ("$(TARGET_OS)","windows")
  LOCAL_LDLIBS += -lws2_32
endif

include $(BUILD_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := aac-dump
LOCAL_DESCRIPTION := AAC bitstream dump tool
LOCAL_CATEGORY_PATH := libs/aac
LOCAL_CFLAGS := -std=gnu99
LOCAL_SRC_FILES := \
	tools/aac_dump.c
LOCAL_LIBRARIES := \
	json \
	libaac \
	libulog
include $(BUILD_EXECUTABLE)

ifdef TARGET_TEST

include $(CLEAR_VARS)

LOCAL_MODULE := tst-libaac
LOCAL_LIBRARIES := \
	libaac \
	libaudio-defs \
	libcunit\
	libulog
LOCAL_CFLAGS := -std=gnu11
LOCAL_SRC_FILES := \
	tests/aac_test_asc_adts.c \
	tests/aac_test_str.c \
	tests/aac_test.c

include $(BUILD_EXECUTABLE)

endif
