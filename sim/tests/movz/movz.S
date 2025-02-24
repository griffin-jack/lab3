###############################################################################
# File         : movn.S
# Project      : EE180 Lab 2: MIPS
#
# Standards/Formatting:
#   MIPS gas, soft tab, 80 column
#
# Description:
#   Test the functionality of the 'movn' instruction.
#
###############################################################################

    .section .boot, "x"
    .balign 4
    .set    noreorder
    .global boot
    .ent    boot
boot:
    lui     $s0, 0x8002         # Load base address for status/test registers
    ori     $s1, $0, 1          # Prepare the 'done' status

    lui     $t0, 0xdeaf         # Prepare a test value: 0xdeafbeef
    nop
    ori     $t0, $t0, 0xbeef

    # Case 1: Condition is zero --> MOVZ should move.
    ori     $t1, $0, 0          # Set condition register to 0
    movz    $t2, $t0, $t1       # Since $t1==0, $t2 should become 0xdeafbeef

    # Case 2: Condition is nonzero --> MOVZ should not move.
    ori     $t1, $0, 1          # Set condition register to nonzero
    ori     $t3, $0, 0          # Initialize $t3 to 0
    movz    $t3, $t0, $t1       # Since $t1!=0, $t3 should remain 0

    # Verify the results:
    subu    $t4, $t2, $t0       # $t4 should be 0 if MOVZ moved in case 1
    subu    $t5, $t3, $0        # $t5 should be 0 if $t3 remains unchanged

    sltiu   $v0, $t4, 1         # $v0 = 1 if $t4 < 1 (i.e. $t4 == 0)
    sltiu   $v1, $t5, 1         # $v1 = 1 if $t5 < 1 (i.e. $t5 == 0)
    and     $v0, $v0, $v1       # $v0 = 1 if both cases are correct

    nop
    sw      $v0, 4($s0)         # Store test result (1 means pass)
    sw      $s1, 0($s0)         # Store 'done'

$done:
    j       $done
    nop

    .end boot
