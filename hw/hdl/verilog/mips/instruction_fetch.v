//=============================================================================
// EE180 Lab 3
//
// Instruction fetch module. Maintains PC and updates it. Reads from the
// instruction ROM.
//=============================================================================

module instruction_fetch (
    input clk,
    input rst,
    input en,
    input jump_target,
    input jump_reg,         // ADDED BY GRAHAM, Signal for jr or jalr forwarded by ID stage
    input [31:0] jr_pc_if,  // ADDED BY GRAHAM, Jump address for jr/jalr forwarded by ID stage
    input [31:0] pc_id,
    input [25:0] instr_id,  // Lower 26 bits of the instruction

    output [31:0] cur_pc_id_out,  // ADDED BY GRAHAM, Pass the current PC to the decode stage, needed for jal & jalr to compute return address (cur_pc + 8)
    output [31:0] pc
);


    wire [31:0] pc_id_p4 = pc_id + 3'h4;
    wire [31:0] j_addr = {pc_id_p4[31:28], instr_id[25:0], 2'b0};

                                                       // EDITED BY GRAHAM
    wire [31:0] pc_next = (jump_target) ? j_addr :     // For j, jal
                          (jump_reg     ? jr_pc_if :   // For jr, jalr (use register value) forwarded from decode stage
                          pc_id_p4);                    // Next sequential instruction


    assign cur_pc_id_out = pc;   //passes the current pc through to ID stage to compute return address for jal & jalr

    dffare #(32) pc_reg (.clk(clk), .r(rst), .en(en), .d(pc_next), .q(pc));


    //cur pc stuff is for jal and jalr, focus on j and jr for now

endmodule
