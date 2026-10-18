set pagination off
set confirm off
set remotetimeout 10
target extended-remote localhost:61240
monitor reset
load
monitor reset
tbreak main
continue
printf "ENTRY_MAIN_PC=0x%x\n", $pc
backtrace 1
delete breakpoints
detach
echo MCU_DETACHED_RUNNING\n
quit
