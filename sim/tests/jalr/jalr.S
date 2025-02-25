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
    .set    noreorder
    .global boot
    .ent    boot
boot:
    lui     $s0, 0x8002         # $s0 = 0x80020000: base address for test/status registers
    # Load the address of the subroutine into $t0.
    # If the assembler supports "la", use it. Otherwise, you may need to use lui/ori.
    la      $t0, subroutine     # $t0 = address of subroutine
    jalr    $t0                # Jump and Link Register: jump to subroutine, save return address (PC+8) into $ra
    nop                        # Delay slot (can be a NOP or useful instruction)

    # When control returns from the subroutine:
    ori     $s1, $zero, 1       # Prepare success value 1 in $s1
    sw      $s1, 4($s0)         # Store 1 at offset 4 (test result)
    sw      $s1, 0($s0)         # Store 1 at offset 0 (done signal)

done:
    j       done               # Infinite loop to halt execution

subroutine:
    # Subroutine: perform an example operation
    addiu   $v0, $zero, 42      # Load 42 into $v0 to simulate work
    jr      $ra                # Return using the link register

    .end boot
