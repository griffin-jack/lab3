###############################################################################
# File         : lbu.S
# Project      : EE180 Lab 2: MIPS
#
# Standards/Formatting:
#   MIPS gas, soft tab, 80 column
#
# Description:
#   Test the functionality of the 'lbu' instruction.
#
###############################################################################


    .section .boot, "x"
    .balign 4
    .set    noreorder
    .global boot
    .ent    boot
boot:
    lui     $s0, 0x8002       # Base address for status/test registers
    ori     $s1, $0, 1        # 'Done' status

    # Use a data memory region (assumed accessible)
    lui     $t0, 0x1000       # $t0 = 0x10000000, base address for our test data
    ori     $t0, $t0, 0       # Clear lower bits

    # Prepare a halfword value to test sign extension.
    # 'ori' zero-extends immediate, so $t1 becomes 0x00008001.
    # In little-endian, a halfword store at an offset of 0 puts the value in the lower half.
    ori     $t1, $0, 0x8001   # $t1 = 0x00008001

    # Store the halfword at offset 0; in little-endian, this stores 0x8001 in the lower 16 bits.
    sh      $t1, 0($t0)       

    # Load the halfword using LH; it should sign-extend 0x8001 to 0xFFFF8001.
    lh      $t2, 0($t0)

    # Prepare the expected value in $t3: 0xFFFF8001.
    lui     $t3, 0xffff       # $t3 = 0xFFFF0000
    ori     $t3, $t3, 0x8001   # $t3 = 0xFFFF8001

    subu    $t4, $t2, $t3     # $t4 should be 0 if $t2 equals expected

    sltiu   $v0, $t4, 1       # $v0 = 1 if $t4 == 0 (test passes)

    sw      $v0, 4($s0)       # Store test result (1 means pass)
    sw      $s1, 0($s0)       # Set 'done'

$done:
    j       $done

    .end boot
