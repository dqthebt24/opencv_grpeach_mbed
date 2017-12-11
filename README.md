## opencv\_grpeach\_mbed ##

This is using opencv on mbed os 5 without cross compile.

The demo include face detection, face recognition, hand gesture detection, motion detection.

### Requieres ###

- GR PEACH board + Camera MT9V111 + GR PEACH LCD 4.3inch. More detail [here](http://gadget.renesas.com/en/product/peach.html)

- Mbed OS 5.4.7, I used the OS [here](https://github.com/dqthebt24/mbed-os-5.4.7)

- I tested on mbed-cli offline and mbed online, for mbed offline, you just clone then build, for mbed online, you can import [here](https://os.mbed.com/users/thedo/code/gr-peach-opencv-project/)

### How to use ###

- Select demo and using it

- On hand gesture detection, click **Sampling** to create sample, then click **Stop** to start demo. You need a black background for good demo. 

- On face recognition click **Add person** then choose ID for face.

**Note:** 
Change heapsize in file 

*mbed-os/targets/TARGET_RENESAS/TARGET_RZ_A1H/device/TOOLCHAIN_GCC_ARM/startup_RZ1AH.S*
 
at line 99 or in file 

*mbed-os/targets/TARGET_RENESAS/TARGET_RZ_A1H/device/TOOLCHAIN_GCC_STD/startup_RZ1AH.S* at line 90 to

```
.EQU    Heap_Size       , 0x00300000
```

to use opencv.


 