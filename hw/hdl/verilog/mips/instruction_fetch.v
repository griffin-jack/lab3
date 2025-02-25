//=============================================================================
// EE180 Lab 3
//
// Instruction fetch module. Maintains PC and updates it. Reads from the
// instruction ROM.
//=============================================================================



//JACK: THIS IS WORKING WELL, DON'T REMOVE ANYTHING ONLY ADD BRANCH CONTROL SIGNALS

module instruction_fetch (
    input clk,
    input rst,
    input en,
    input [31:0] jump_target,

    output [31:0] pc
);

    wire isJump = ~(jump_target == 32'b0);
    wire [31:0] pc_next = isJump ? jump_target : (pc + 3'h4);
   

    dffare #(32) pc_reg (.clk(clk), .r(rst), .en(en), .d(pc_next), .q(pc));

endmodule
