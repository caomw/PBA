LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := pba
LOCAL_SRC_FILES := libpba.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)

LOCAL_MODULE    := test
LOCAL_SRC_FILES := test.cpp
LOCAL_STATIC_LIBRARIES += pba
LOCAL_LDLIBS +=  -llog -ldl
include $(BUILD_SHARED_LIBRARY)
