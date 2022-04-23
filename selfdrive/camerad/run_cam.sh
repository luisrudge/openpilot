#! /bin/sh
LD_PRELOAD=/data/android_hook/ldpreloadhook/hook.so DEBUG_FRAMES=1 LOGPRINT=debug SEND_ROAD=1 ZMQ=1 ./camerad
