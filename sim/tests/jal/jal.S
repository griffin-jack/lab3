###############################################################################
# File         : j.S
# Project      : EE180 Lab 2: MIPS
#
# Standards/Formatting:
#   MIPS gas, soft tab, 80 column
#
# Description:
#   Test the functionality of the 'j' instruction.
#
###############################################################################


    .section .boot, "x"
    .balign 4
    .set noreorder
    .global boot
    .ent boot
boot:
    lui     $s0, 0x8002         # $s0 = 0x80020000: base address for test registers
    jal     subroutine          # Jump-and-Link: save return address (PC+8) in $ra, then jump
    nop                         # Delay slot (can be a NOP or useful instruction)
    # When we return, execution resumes here.
    ori     $s1, $zero, 1       # Prepare success value 1 in $s1
    sw      $s1, 4($s0)         # Store 1 at offset 4 (test result)
    sw      $s1, 0($s0)         # Store 1 at offset 0 (done signal)

done:
    j       done                # Infinite loop to halt execution

subroutine:
    # Subroutine: simply returns.
    addiu   $v0, $zero, 42      # (Optional) Load a value into $v0 to simulate work
    jr      $ra                 # Return to caller using the link register
                     # Delay slot
    .end boot