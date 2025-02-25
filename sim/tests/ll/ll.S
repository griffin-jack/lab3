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
    .set noreorder
    .global boot
    .ent boot
boot:
    lui     $s0, 0x8002         # $s0 = 0x80020000 (base address for test registers)
    li      $t1, 1              # Prepare constant 1 in $t1

atomic_store:
    ll      $t0, 4($s0)         # Load Linked from address 0x80020004
    sc      $t1, 4($s0)         # Attempt to Store Conditional: if successful, $t1 becomes 1; if not, $t1 becomes 0
    beq     $t1, $zero, atomic_store  # If SC failed ($t1 == 0), retry

    # At this point, the SC succeeded and address 0x80020004 contains 1.
    sw      $t1, 0($s0)         # Signal "done" by storing 1 to address 0x80020000

done:
    j       done               # Infinite loop
    .end boot