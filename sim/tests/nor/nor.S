###############################################################################
# File         : ori.S
# Project      : EE180 Lab 2: MIPS
#
# Standards/Formatting:
#   MIPS gas, soft tab, 80 column
#
# Description:
#   Test the functionality of the 'ori' instruction.
#
###############################################################################

    .section .boot, "x"
    .balign 4
    .set    noreorder
    .global boot
    .ent    boot
boot:
    lui     $s0, 0x8002         # Load the base address of the status/test registers
    lui     $t1, 0xffff
    ori     $t2, 0xffff          # Prepare the 'done' status
    nor     $t3, $t1, $t2         # Prepare the 'done' status
    sltiu   $s1, $t3, 1
    nop 
    nop
    sw      $s1, 4($s0)         # Set the test result
    sw      $s1, 0($s0)         # Set 'done'

$done:
    j       $done


    .end boot
