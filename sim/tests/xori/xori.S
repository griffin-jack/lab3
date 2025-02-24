###############################################################################
# File         : xor.S
# Project      : EE180 Lab 2: MIPS
#
# Standards/Formatting:
#   MIPS gas, soft tab, 80 column
#
# Description:
#   Test the functionality of the 'xor' instruction.
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
    lui     $t0, 0xdeaf         # A = 0xdeafbeef
    ori     $t0, $t0, 0xbeef
    lui     $t1, 0x3141         # B = 0x31415926
    ori     $t1, $t1, 0x5926
    lui     $t2, 0xefee         # C = 0xefeee7c8
    ori     $t2, $t2, 0xe7c8
    xor     $t3, $t0, $t1       # D = xor(A,B) = 0xefeee7c9
    xor     $t4, $t2, $t3       # E = xor(C,D) = 0x1

    #Graham's expanded testing
    xori    $t5, $0, 1          # $t5 = 1
    xori    $t6, $t5, 1         # $t6 = 0
    xori    $t7, $t6, 1         # $t7 = 1
    and     $t7, $t7, $t4       # $t7 still = 1

    sw      $t7, 4($s0)         # Set the test result
    sw      $s1, 0($s0)         # Set 'done'

$done:
    j       $done

    .end boot
