
# Intel mSGDMA kernel DMA driver (Intel/Altera Streaming mode)
**Altera mSGDMA DMA** kernel driver module supports multiple instances and can be used in write or read mode.
Designed and optimized for high speed transmissions using **Altera Streaming ST®** protocol on **FPGA** side. Originally intended for **Software Defined Radio / SDR**.

## Driver features:  
- tested on **Cyclone® V SOC FPGA** and **Arria® 10 SOC FPGA** 
- Linux kernel 4.x and 5.x tree supported (tested with 4.12, 4.17, 4.20 and 5.4) 
- configuration via **Device Tree Overlay** (e.g. registers, wr/rd mode, interrupts)
- multiple instances supported
- internally uses **dma_map_single()** for dynamic DMA mapping
- Altera MSGDMA IP Memory-Mapped to Streaming [ST --> MM] and [MM --> ST]
- write device: `/dev/altera_msgdma_wrX`
- read device: `/dev/altera_msgdma_rdX`
- extended **IOCTL** calls supported

## IOCTL calls:
- `IO_ALTERA_MSGDMA_SET_BUFFER_SIZE` - sets driver internal buffer size
- `IO_ALTERA_MSGDMA_SET_MIN_BLOCK_SIZE` - sets min. transfer block size 
- `IO_ALTERA_MSGDMA_STOP` - stops MSGDMA core from processing the next descriptor
- `IO_ALTERA_MSGDMA_RESET` - resets MSGDMA core
- `IO_ALTERA_MSGDMA_STOP_RESET` - combination of previous `'IO_ALTERA_MSGDMA_STOP`' and `IO_ALTERA_MSGDMA_RESET` calls 

## Build
### Option 1] - cross compile 
- set `KERNEL_SRC_DIR` in Makefile (e.g. `KERNEL_SRC_DIR ?= $(HOME)/build_sockit_arm/socfpga-kernel-dev-3/KERNEL`)
- for cross compilation is necessary to export **CC variable** (gcc)
- tested with: **gcc-linaro-7.2.1-2017.11-x86_64_arm-linux-gnueabihf** 
- e.g. `export CC=`pwd`/socfpga-kernel-dev-3/dl/gcc-linaro-7.2.1-2017.11-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-`
- test cross compiler with `${CC}gcc --version`
- `make clean && make` 

### Option 2] -  compilation on target embedded Linux system
- install appropriate kernel headers
- repair headers install (see: https://github.com/armbian/build/issues/74; run `make scripts`; for Linux 5.x run `sudo make M=scripts/mod` ) 
- `make clean && make` 

## Device Tree Overlay
 >Cyclone® V SOC FPGA
- full FPGA reconfiguration is supported (use your FPGA bitstream and set firmware-name="*.rbf")
- for **Cyclone® V** target-path is `"/soc/fpga-region0"`
```
/dts-v1/; /plugin/;
/ {
        fragment@0 {
        target-path="/soc/fpga-region0";
        #address-cells=<1>;
        #size-cells=<1>;
        __overlay__ {
                firmware-name="altera_msgdma_socfpga.rbf";
                #address-cells=<1>;
                #size-cells=<1>;
                
                msgdma_streaming_write: msgdma@c0000000 {
                            compatible = "altr,altera-msgdma-st-1.0";
                            reg = <0xc0000000 0x00000020>,
                                <0xc0000020 0x00000010>;
                            reg-names = "csr", "descriptor_slave";
                            string-property = "msgdma_write";
                            interrupts = <0 40 4>;
                            clocks = <&clk_0>;
                        };

                msgdma_streaming_read: msgdma@c0001000 {
                            compatible = "altr,altera-msgdma-st-1.0";
                            reg = <0xc0001000 0x00000020>,
                                <0xc0001020 0x00000010>;
                            reg-names = "csr", "descriptor_slave";
                            string-property = "msgdma_read";
                            interrupts = <0 41 4>;
                            clocks = <&clk_0>;
                        };
                
                };
        };
};

```
>Arria® 10 SOC FPGA 
- full reconfiguration is not allowed on Arria 10  - use **u-boot**
- for **Arria ® 10** target-path is `"/soc"`
- modify reg. settings for your requirements 
- modify interrupts settings for your requirements (numbers for Cyclone V and Arria 10 are different !!!) 
- set msgdma_write or msgdma_read property

 ```
 /* dtc -O dtb -o altera_msgdma_arria10.dtbo -b 0 -@ altera_msgdma_arria10.dts */
/dts-v1/; /plugin/;
/ {
        fragment@0 {
        /* target-path="/soc/base_fpga_region"; */ 
        target-path="/soc";
        #address-cells=<1>;
        #size-cells=<1>;
        __overlay__ {
                /*
                 
                https://forum.rocketboards.org/t/problems-loading-device-tree-overlays-on-arria10-socfpga-kernel-4-14-126/2145/3
                https://www.intel.co.jp/content/www/jp/ja/programmable/documentation/pne1482303525167.html
                https://forum.rocketboards.org/t/arria-10-fpga-manager-driver-doesnt-support-full-reconfiguration/711/3
                */
                /* --> Full reconfiguration is not allowed on Arria 10 - use u-boot:  firmware-name="ghrd_10as066n2.core.rbf"; */
                
        
                #address-cells=<1>;
                #size-cells=<1>;
                
                clk_0: clk_0 {
                        compatible = "fixed-clock";
                        #clock-cells = <0>;
                        clock-frequency = <100000000>;	/* 100.00 MHz */
                        clock-output-names = "clk_0";
                        }; 
                
                /*
                 * ----------------   
                 * Write MSGDMA / 0
                 * ----------------
                 */
                msgdma_streaming_write_0: msgdma@c0040000 {
                            compatible = "altr,altera-msgdma-st-1.0";
                            reg = <0xc0040000 0x00000020>,
                                  <0xc0040020 0x00000010>;
                            reg-names = "csr", "descriptor_slave";
                            string-property = "msgdma_write";
                            interrupts = <0 21 4>;
                            interrupt-parent = <&intc>;
                            clocks = <&clk_0>;
                        };
                /*
                 * ---------------   
                 * READ MSGDMA / 0
                 * ---------------
                 */
                msgdma_streaming_read_0: msgdma@c0041000 {
                            compatible = "altr,altera-msgdma-st-1.0";
                            reg = <0xc0041000 0x00000020>,
                                  <0xc0041020 0x00000010>;
                            reg-names = "csr", "descriptor_slave";
                            string-property = "msgdma_read";
                            interrupts = <0 22 4>;
                            interrupt-parent = <&intc>;
                            clocks = <&clk_0>;
                        };
                

        };
};
 ```
> registers ranges are from Quartus QSys/Platform Designer

![Quartus](https://github.com/pavelfpl/altera_msgdma_st/blob/master/qsys_platform_designer.png)

## MISC
- **DTC compilator** is used for compilation of **Device Tree Blob** (use newer DTC like 1.4.6/1.4.7) 
- run: `dtc -O dtb -o altera_msgdma_rbf.dtbo -b 0 -@ altera_msgdma_rbf.dts`
- non root access: see `99-altera_msgdma.rules`

## Benchmarks 
- **Arria® 10, HPS configured with 100MHz master clock**

  Total Write/read bytes count: 40960000  
  Elapsed time: 648.864000 ms.  
  Combined speed W/R [MBytes/s]: 63.13  
  Combined speed W/R [Mbit/s]: 505.01  
  Single direction speed [Mbit/s]: 1010.01  

