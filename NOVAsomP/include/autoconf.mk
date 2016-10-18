CONFIG_SYS_MMCSD_FS_BOOT_PARTITION=y
CONFIG_SYS_PL310_BASE="L2_PL310_BASE"
CONFIG_CMD_SOURCE=y
CONFIG_VIDEO_BMP_LOGO=y
CONFIG_BOOTM_NETBSD=y
CONFIG_BOOTCOMMAND="mmc dev ${mmcdev}; if mmc rescan; then if run mmcloadbootscript; then run bootscript; else if run mmcloadimage; then if run mmcloadfdt; then if run mmcloadinitrd; then run board_boot; fi; fi; fi; fi; fi; usb start;if run usbloadbootscript; then run bootscript; else if run usbloadimage; then if run usbloadfdt; then if run usbloadinitrd; then run board_boot; fi; fi; fi; fi; fi; run netboot;"
CONFIG_BOARD_EARLY_INIT_F=y
CONFIG_VGA_AS_SINGLE_DEVICE=y
CONFIG_MXC_USB_FLAGS=0
CONFIG_IMX_CONFIG="arch/arm/imx-common/spl_sd.cfg"
CONFIG_CMD_ITEST=y
CONFIG_BOOTM_VXWORKS=y
CONFIG_CMD_EDITENV=y
CONFIG_CMD_SETEXPR=y
CONFIG_CMD_ENV_EXISTS=y
CONFIG_SYS_LONGHELP=y
CONFIG_SYS_GENERIC_BOARD=y
CONFIG_SYS_LOAD_ADDR=$(CONFIG_LOADADDR)
CONFIG_DISPLAY_BOARDINFO=y
CONFIG_CMD_XIMG=y
CONFIG_CMD_CACHE=y
CONFIG_BOOTDELAY=2
CONFIG_SYS_SATA_MAX_DEVICE=y
CONFIG_SYS_HELP_CMD_WIDTH=8
CONFIG_NR_DRAM_BANKS=y
CONFIG_IMX_VIDEO_SKIP=y
CONFIG_FS_FAT=y
CONFIG_BOOTM_RTEMS=y
CONFIG_SYS_CBSIZE=256
CONFIG_SYS_SPL_MALLOC_SIZE=0x3200000
CONFIG_SYS_MONITOR_LEN="(CONFIG_SYS_U_BOOT_MAX_SIZE_SECTORS/2*1024)"
CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR=138
CONFIG_BOOTM_LINUX=y
CONFIG_DEFAULT_FDT_FILE="novasomp.dtb"
CONFIG_BOARD_LATE_INIT=y
CONFIG_CMD_CONSOLE=y
CONFIG_MII=y
CONFIG_CMD_FAT=y
CONFIG_DWC_AHSATA=y
CONFIG_SYS_THUMB_BUILD=y
CONFIG_SYS_CACHELINE_SIZE=32
CONFIG_MMC=y
CONFIG_REVISION_TAG=y
CONFIG_SYS_FSL_SEC_ADDR="CAAM_BASE_ADDR"
CONFIG_CMD_MISC=y
CONFIG_SPL_LIBCOMMON_SUPPORT=y
CONFIG_ENV_OFFSET="(6 * 64 * 1024)"
CONFIG_MXC_OCOTP=y
CONFIG_MX6=y
CONFIG_USB_MAX_CONTROLLER_COUNT=2
CONFIG_ENV_OVERWRITE=y
CONFIG_CMD_NET=y
CONFIG_CMD_NFS=y
CONFIG_ENV_SIZE="(8 * 1024)"
CONFIG_CMD_PING=y
CONFIG_SYS_L2_PL310=y
CONFIG_VIDEO_BMP_GZIP=y
CONFIG_SYS_MALLOC_LEN="(10 * SZ_1M)"
CONFIG_SYS_MMC_ENV_DEV=0
CONFIG_SYS_I2C_SPEED=100000
CONFIG_SPL_LIBDISK_SUPPORT=y
CONFIG_SYS_TEXT_BASE=0x17800000
CONFIG_CMD_FLASH=y
CONFIG_CMD_SAVEENV=y
CONFIG_MXC_GPT_HCLK=y
CONFIG_MXC_UART=y
CONFIG_SPLASH_SCREEN=y
CONFIG_SYS_BARGSIZE=$(CONFIG_SYS_CBSIZE)
CONFIG_BOOTM_PLAN9=y
CONFIG_SYS_U_BOOT_MAX_SIZE_SECTORS=800
CONFIG_SPL_TEXT_BASE=0x00908000
CONFIG_VIDEO_BMP_RLE8=y
CONFIG_MXC_USB_PORTSC="(PORT_PTS_UTMI | PORT_PTS_PTW)"
CONFIG_BOOTSTAGE_USER_COUNT=20
CONFIG_SYS_FSL_JR0_ADDR="(CAAM_BASE_ADDR + 0x1000)"
CONFIG_IPUV3_CLK=260000000
CONFIG_CMD_MEMORY=y
CONFIG_SYS_MAXARGS=16
CONFIG_BMP_16BPP=y
CONFIG_CMD_RUN=y
CONFIG_SYS_PBSIZE="(CONFIG_SYS_CBSIZE + 128)"
CONFIG_FEC_XCV_TYPE="RMII"
CONFIG_MXC_GPIO=y
CONFIG_ARM_ERRATA_743622=y
CONFIG_BOARDDIR="board/novasomp"
CONFIG_CMD_HDMIDETECT=y
CONFIG_BOUNCE_BUFFER=y
CONFIG_OF_LIBFDT=y
CONFIG_SPL_STACK=0x0091FFB8
CONFIG_PHYLIB=y
CONFIG_BOARD_POSTCLK_INIT=y
CONFIG_CMDLINE_EDITING=y
CONFIG_CMD_USB=y
CONFIG_SYS_CONSOLE_IS_IN_ENV=y
CONFIG_CMD_EXT2=y
CONFIG_USB_EHCI=y
CONFIG_SPL_I2C_SUPPORT=y
CONFIG_CMD_SETGETDCR=y
CONFIG_SPL_GPIO_SUPPORT=y
CONFIG_SYS_MMC_MAX_BLK_COUNT=65535
CONFIG_ZLIB=y
CONFIG_LOADADDR=0x12000000
CONFIG_ETHPRIME="FEC"
CONFIG_CMD_BOOTD=y
CONFIG_CMD_BOOTZ=y
CONFIG_AUTO_COMPLETE=y
CONFIG_MP=y
CONFIG_FSL_USDHC=y
CONFIG_ENV_IS_IN_MMC=y
CONFIG_DOS_PARTITION=y
CONFIG_GZIP=y
CONFIG_CMD_FPGA=y
CONFIG_SYS_FSL_ESDHC_ADDR=0
CONFIG_SYS_INIT_RAM_SIZE="IRAM_SIZE"
CONFIG_VIDEO_IPUV3=y
CONFIG_FEC_MXC_PHYADDR=0
CONFIG_SYS_BAUDRATE_TABLE="{ 9600, 19200, 38400, 57600, 115200 }"
CONFIG_CMD_SATA=y
CONFIG_SPL_LIBGENERIC_SUPPORT=y
CONFIG_SYS_HUSH_PARSER=y
CONFIG_SPL_MMC_SUPPORT=y
CONFIG_VIDEO=y
CONFIG_SYS_SDRAM_BASE="PHYS_SDRAM"
CONFIG_IMAGE_FORMAT_LEGACY=y
CONFIG_SYS_BOOT_RAMDISK_HIGH=y
CONFIG_CFB_CONSOLE=y
CONFIG_SPL_LDSCRIPT="arch/arm/cpu/armv7/omap-common/u-boot-spl.lds"
CONFIG_CMD_BDI=y
CONFIG_PHY_SMSC=y
CONFIG_CMD_DHCP=y
CONFIG_SPL_SERIAL_SUPPORT=y
CONFIG_SYS_FSL_USDHC_NUM=2
CONFIG_CMD_ECHO=y
CONFIG_GENERIC_MMC=y
CONFIG_SYS_INIT_SP_OFFSET="(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)"
CONFIG_SYS_CONSOLE_OVERWRITE_ROUTINE=y
CONFIG_SYS_SPL_MALLOC_START=0x18300000
CONFIG_SYS_I2C=y
CONFIG_SYS_INIT_RAM_ADDR="IRAM_BASE_ADDR"
CONFIG_SPL_BSS_MAX_SIZE=0x100000
CONFIG_SPL_BSS_START_ADDR=0x18200000
CONFIG_SPL_PAD_TO=$(CONFIG_SPL_MAX_SIZE)
CONFIG_EXTRA_ENV_SETTINGS="setenv splashimage_mmc_dev 00setenv splashimage_mmc_part 10splashimage=0x108000000setenv splashpos m,m0splashimage_file_name=splash.bmp.gz0script=boot.scr0image=zImage0initrd=uInitrd0fdt_file=" CONFIG_DEFAULT_FDT_FILE "0console=ttymxc20splashpos=m,m0fdt_high=0xffffffff0initrd_high=0xffffffff0fdt_addr=0x180000000fsaddr=0x128000000boot_fdt=try0ip_dyn=yes0ethaddr=5c:b8:b2:91:9f:290mmcdev=" __stringify(CONFIG_SYS_MMC_ENV_DEV) "0mmcpart=10usbdev=00usbpart=10mmcroot=/dev/mmcblk2p2 rootwait rw0ramroot=/dev/ram rootwait rw0update_sd_firmware_filename=u-boot.imx0update_sd_firmware=if test ${ip_dyn} = yes; then setenv get_cmd dhcp; else setenv get_cmd tftp; fi; if mmc dev ${mmcdev}; then if ${get_cmd} ${update_sd_firmware_filename}; then setexpr fw_sz ${filesize} / 0x200; setexpr fw_sz ${fw_sz} + 1; mmc write ${loadaddr} 0x2 ${fw_sz}; fi; fi0bootscript=echo Running bootscript from mmc ...; source0mmcloadbootscript=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${script};0mmcloadimage=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${image}0mmcloadinitrd=fatload mmc ${mmcdev}:${mmcpart} ${fsaddr} ${initrd}0mmcloadfdt=fatload mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdt_file}0usbloadbootscript=fatload usb ${usbdev}:${usbpart} ${loadaddr} ${script};0usbloadimage=fatload usb ${usbdev}:${usbpart} ${loadaddr} ${image}0usbloadinitrd=fatload usb ${usbdev}:${usbpart} ${fsaddr} ${initrd}0usbloadfdt=fatload usb ${usbdev}:${usbpart} ${fdt_addr} ${fdtfile}0boardargs=setenv bootargs console=${console},${baudrate} root=${ramroot} video=mxcfb0:dev=hdmi,1280x720M@60,if=RGB24 fbmem=28M ramdisk_size=96000;0board_boot=echo Booting ...; run boardargs; bootz ${loadaddr} ${fsaddr} ${fdt_addr};0netargs=setenv bootargs console=${console},${baudrate} root=/dev/nfs ip=dhcp nfsroot=${serverip}:${nfsroot},v3,tcp0netboot=echo Booting from net ...; run netargs; if test ${ip_dyn} = yes; then setenv get_cmd dhcp; else setenv get_cmd tftp; fi; ${get_cmd} ${image}; if test ${boot_fdt} = yes || test ${boot_fdt} = try; then if ${get_cmd} ${fdt_addr} ${fdtfile}; then bootz ${loadaddr} - ${fdt_addr}; else if test ${boot_fdt} = try; then bootz; else echo WARN: Cannot load the DT; fi; fi; else bootz; fi;0"
CONFIG_GPT_TIMER=y
CONFIG_SYS_INIT_SP_ADDR="(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)"
CONFIG_FSL_ESDHC=y
CONFIG_SYS_VIDEO_LOGO_MAX_SIZE="(1 << 20)"
CONFIG_IMX_THERMAL=y
CONFIG_BAUDRATE=115200
CONFIG_CMD_BMODE=y
CONFIG_CMDLINE_TAG=y
CONFIG_MXC_UART_BASE="UART3_BASE"
CONFIG_SPLASH_SCREEN_ALIGN=y
CONFIG_INITRD_TAG=y
CONFIG_PARTITIONS=y
CONFIG_SYS_MEMTEST_END="(CONFIG_SYS_MEMTEST_START + 500 * SZ_1M)"
CONFIG_CMD_I2C=y
CONFIG_LIBATA=y
CONFIG_FEC_MXC=y
CONFIG_SYS_NO_FLASH=y
CONFIG_SYS_DEF_EEPROM_ADDR=0
CONFIG_FS_EXT4=y
CONFIG_DWC_AHSATA_PORT_ID=0
CONFIG_SPL_MAX_SIZE=0x10000
CONFIG_MACH_TYPE="MACH_TYPE_NOVASOMP"
CONFIG_SPL_FRAMEWORK=y
CONFIG_MX6QDL=y
CONFIG_SYS_PROMPT="=> "
CONFIG_USB_STORAGE=y
CONFIG_USB_EHCI_MX6=y
CONFIG_IMX_HDMI=y
CONFIG_DISPLAY_CPUINFO=y
CONFIG_SETUP_MEMORY_TAGS=y
CONFIG_SPL_FAT_SUPPORT=y
CONFIG_SYS_MEMTEST_START=0x10000000
CONFIG_CMD_LOADB=y
CONFIG_CMD_LOADS=y
CONFIG_CMD_IMI=y
CONFIG_CONS_INDEX=y
CONFIG_LMB=y
CONFIG_SYS_I2C_MXC=y
CONFIG_DWC_AHSATA_BASE_ADDR="SATA_ARB_BASE_ADDR"
CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG=y
CONFIG_LBA48=y
CONFIG_CMD_MII=y
CONFIG_CMD_BMP=y
CONFIG_VIDEO_LOGO=y
CONFIG_CMD_MMC=y
CONFIG_CMD_FUSE=y
CONFIG_SPL_FS_LOAD_PAYLOAD_NAME="u-boot.img"
