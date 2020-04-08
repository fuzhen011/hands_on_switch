let g:ale_c_gcc_executable = 'arm-none-eabi-gcc'
let g:ale_cpp_gcc_executable = 'arm-none-eabi-gcc'
let g:ale_c_gcc_options = '-g -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 -DNVM3_DEFAULT_MAX_OBJECT_SIZE=512 -DENABLE_LOGGING=1 -D__HEAP_SIZE=0x1700 -DMESH_LIB_NATIVE=1 -DHAL_CONFIG=1 -D__STACK_SIZE=0x1000 -DNVM3_DEFAULT_NVM_SIZE=24576 -DEFR32MG12P332F1024GL125=1 -I$HOME/work/projs/btmesh_hands_on/hands_on_switch/platform/emlib/src -I$HOME/work/projs/btmesh_hands_on/hands_on_switch/protocol/bluetooth/bt_mesh/inc -I$HOME/work/projs/btmesh_hands_on/hands_on_switch/platform/CMSIS/Include -I$HOME/work/projs/btmesh_hands_on/hands_on_switch/platform/emlib/inc -I$HOME/work/projs/btmesh_hands_on/hands_on_switch/platform/emdrv/nvm3/src -I$HOME/work/projs/btmesh_hands_on/hands_on_switch/hardware/kit/common/bsp/thunderboard -I$HOME/work/projs/btmesh_hands_on/hands_on_switch/hardware/kit/common/drivers -I$HOME/work/projs/btmesh_hands_on/hands_on_switch/hardware/kit/EFR32MG12_BRD4166A/config -I$HOME/work/projs/btmesh_hands_on/hands_on_switch/platform/radio/rail_lib/chip/efr32/efr32xg1x -I$HOME/work/projs/btmesh_hands_on/hands_on_switch/platform/Device/SiliconLabs/EFR32MG12P/Include -I$HOME/work/projs/btmesh_hands_on/hands_on_switch/platform/emdrv/nvm3/inc -I$HOME/work/projs/btmesh_hands_on/hands_on_switch/platform/emdrv/sleep/inc -I$HOME/work/projs/btmesh_hands_on/hands_on_switch/platform/bootloader/api -I$HOME/work/projs/btmesh_hands_on/hands_on_switch/platform/emdrv/common/inc -I$HOME/work/projs/btmesh_hands_on/hands_on_switch/platform/common/inc -I$HOME/work/projs/btmesh_hands_on/hands_on_switch/protocol/bluetooth/bt_mesh/inc/soc -I$HOME/work/projs/btmesh_hands_on/hands_on_switch/hardware/kit/common/halconfig -I$HOME/work/projs/btmesh_hands_on/hands_on_switch/platform/emdrv/uartdrv/inc -I$HOME/work/projs/btmesh_hands_on/hands_on_switch/protocol/bluetooth/bt_mesh/inc/common -I$HOME/work/projs/btmesh_hands_on/hands_on_switch/protocol/bluetooth/bt_mesh/src -I$HOME/work/projs/btmesh_hands_on/hands_on_switch/protocol/bluetooth/ble_stack/src/soc -I$HOME/work/projs/btmesh_hands_on/hands_on_switch/platform/radio/rail_lib/plugin/coexistence/hal/efr32 -I$HOME/work/projs/btmesh_hands_on/hands_on_switch/platform/radio/rail_lib/plugin/coexistence/common -I$HOME/work/projs/btmesh_hands_on/hands_on_switch/protocol/bluetooth/ble_stack/inc/soc -I$HOME/work/projs/btmesh_hands_on/hands_on_switch/platform/emdrv/sleep/src -I$HOME/work/projs/btmesh_hands_on/hands_on_switch/platform/radio/rail_lib/common -I$HOME/work/projs/btmesh_hands_on/hands_on_switch/hardware/kit/common/bsp -I$HOME/work/projs/btmesh_hands_on/hands_on_switch/hardware/kit/EFR32MG12_BRD4166A/config -I$HOME/work/projs/btmesh_hands_on/hands_on_switch/platform/service/sleeptimer/src -I$HOME/work/projs/btmesh_hands_on/hands_on_switch/platform/halconfig/inc/hal-config -I$HOME/work/projs/btmesh_hands_on/hands_on_switch/platform/radio/rail_lib/protocol/ieee802154 -I$HOME/work/projs/btmesh_hands_on/hands_on_switch/platform/service/sleeptimer/inc -I$HOME/work/projs/btmesh_hands_on/hands_on_switch/platform/emdrv/gpiointerrupt/inc -I$HOME/work/projs/btmesh_hands_on/hands_on_switch/platform/Device/SiliconLabs/EFR32MG12P/Source -I$HOME/work/projs/btmesh_hands_on/hands_on_switch/platform/service/sleeptimer/config -I$HOME/work/projs/btmesh_hands_on/hands_on_switch/platform/radio/rail_lib/protocol/ble -I$HOME/work/projs/btmesh_hands_on/hands_on_switch/platform/Device/SiliconLabs/EFR32MG12P/Source/GCC -I$HOME/work/projs/btmesh_hands_on/hands_on_switch/platform/emdrv/gpiointerrupt/src -I$HOME/work/projs/btmesh_hands_on/hands_on_switch/platform/emdrv/dmadrv/inc -I$HOME/work/projs/btmesh_hands_on/hands_on_switch/platform/emdrv/dmadrv/src -I$HOME/work/projs/btmesh_hands_on/hands_on_switch -I$HOME/work/projs/btmesh_hands_on/hands_on_switch/platform/radio/rail_lib/plugin -Os -fno-builtin -Wall -fmessage-length=0 -ffunction-sections -fdata-sections -mfpu=fpv4-sp-d16 -mfloat-abi=softfp'
