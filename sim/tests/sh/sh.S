###############################################################################
# File         : sub.S
# Project      : EE180 Lab 2: MIPS
#
# Standards/Formatting:
#   MIPS gas, soft tab, 80 column
#
# Description:
#   Test the functionality of the 'sub' instruction.
#
###############################################################################


    .section .boot, "x"
    .balign 4
    .set    noreorder
    .global boot
    .ent    boot
boot:
    lui     $s0, 0x8002         # Load the base address of the status/test registers
    ori     $s1, $0, 1          # Prepare the 'done' status
    sh      $s1, 2($s0)         # Store 1 into lower half of word at 0x80020000
    sh      $s1, 6($s0)         # Store 1 into lower half of word at 0x80020004

$done:
    j       $done

    .end boot
