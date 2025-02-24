//=============================================================================
// EE180 Lab 3
//
// Decode module. Determines what to do with an instruction.
//=============================================================================

`include "mips_defines.v"

module decode (
    input [31:0] pc,          // program counter for next instruction
    input [31:0] cur_pc,      //ADDED BY GRAHAM
    input [31:0] instr,       // current instructions program counter, used to compute return address
    input [31:0] rs_data_in,
    input [31:0] rt_data_in,

    output wire [4:0] reg_write_addr,
    output wire jump_branch,
    output wire jump_target,
    output wire jump_reg,
    output wire [31:0] jr_pc,
    output reg [3:0] alu_opcode,
    output wire [31:0] alu_op_x,
    output wire [31:0] alu_op_y,
    output wire mem_we,
    output wire [31:0] mem_write_data,
    output wire mem_read,
    output wire mem_byte,
    output wire mem_halfword_ex,        //ADDED BY GRAHAM for SH instruction only
    output wire mem_signextend,
    output wire reg_we,
    output wire movn,
    output wire movz,
    output wire [4:0] rs_addr,
    output wire [4:0] rt_addr,
    output wire atomic_id,
    input  atomic_ex,
    output wire mem_sc_mask_id,
    output wire mem_sc_id,

    output wire stall,

    input reg_we_ex,
    input [4:0] reg_write_addr_ex,
    input [31:0] alu_result_ex,
    input mem_read_ex,

    input reg_we_mem,
    input [4:0] reg_write_addr_mem,
    input [31:0] reg_write_data_mem
);


//DONE BY GRAHAM
//  - For jr and jalr: 
//      - Have set jr_pc to rs_data in decode
//      - set stall = 1 if isJumpReg
//      - pass jr_pc to IF stage
//      - update pc = jr_pc if isJumpReg in IF stage
//  - for jal and jalr:
//      - have set reg_write_addr to either $ra or $rd
//      - have set alu_op_x to return_address (cur_pc + 8)
//      - have passed cur_pc from IF to ID stage
//      - have ensured that reg write is enabled
//      - with the proper reg_we and reg_write_addr, the writeback stage will handle the rest
//  - j already implemented
//  ___________________________
//  - Added zero extension for relevant I-type instructions
//  - updated alu_op_y to use zero extended immediates
//  - added alu_opcode decodes and alu operations for XOR, XORI, 


//******************************************************************************
// instruction field
//******************************************************************************

    wire [5:0] op = instr[31:26];
    assign rs_addr = instr[25:21];
    assign rt_addr = instr[20:16];
    wire [4:0] rd_addr = instr[15:11];
    wire [4:0] shamt = instr[10:6];
    wire [5:0] funct = instr[5:0];
    wire [15:0] immediate = instr[15:0];

    wire [31:0] rs_data, rt_data;

//******************************************************************************
// branch instructions decode
//******************************************************************************

    wire isBEQ    = (op == `BEQ);
    wire isBGEZNL = (op == `BLTZ_GEZ) & (rt_addr == `BGEZ);
    wire isBGEZAL = (op == `BLTZ_GEZ) & (rt_addr == `BGEZAL);
    wire isBGTZ   = (op == `BGTZ) & (rt_addr == 5'b00000);
    wire isBLEZ   = (op == `BLEZ) & (rt_addr == 5'b00000);
    wire isBLTZNL = (op == `BLTZ_GEZ) & (rt_addr == `BLTZ);
    wire isBLTZAL = (op == `BLTZ_GEZ) & (rt_addr == `BLTZAL);
    wire isBNE    = (op == `BNE);
    wire isBranchLink = (isBGEZAL | isBLTZAL);


//******************************************************************************
// jump instructions decode
//******************************************************************************

    wire isJ    = (op == `J);
    wire isJAL  = (op == `JAL);                         //ADDED BY GRAHAM
    wire isJR   = (op == `SPECIAL) & (funct == `JR);    //ADDED BY GRAHAM
    wire isJALR = (op == `SPECIAL) & (funct == `JALR);  //ADDED BY GRAHAM

//******************************************************************************
// shift instruction decode
//******************************************************************************

    wire isSLL = (op == `SPECIAL) & (funct == `SLL);
    wire isSRL = (op == `SPECIAL) & (funct == `SRL);
    wire isSRA = (op == `SPECIAL) & (funct == `SRA);
    wire isSLLV = (op == `SPECIAL) & (funct == `SLLV);
    wire isSRLV = (op == `SPECIAL) & (funct == `SRLV);

    wire isShiftImm = isSLL | isSRL | isSRA;
    wire isShift = isShiftImm | isSLLV | isSRLV;

//******************************************************************************
// ALU instructions decode / control signal for ALU datapath
//******************************************************************************

    always @* begin
        casex({op, funct})
            {`ADDI, `DC6}:      alu_opcode = `ALU_ADD;
            {`ADDIU, `DC6}:     alu_opcode = `ALU_ADDU;
            {`SLTI, `DC6}:      alu_opcode = `ALU_SLT;
            {`SLTIU, `DC6}:     alu_opcode = `ALU_SLTU;
            {`ANDI, `DC6}:      alu_opcode = `ALU_AND;
            {`ORI, `DC6}:       alu_opcode = `ALU_OR;
            {`LB, `DC6}:        alu_opcode = `ALU_ADD;
            {`LW, `DC6}:        alu_opcode = `ALU_ADD;
            {`LBU, `DC6}:       alu_opcode = `ALU_ADD;
            {`SB, `DC6}:        alu_opcode = `ALU_ADD;
            {`SW, `DC6}:        alu_opcode = `ALU_ADD;
            {`BEQ, `DC6}:       alu_opcode = `ALU_SUBU;
            {`BNE, `DC6}:       alu_opcode = `ALU_SUBU;
            {`SPECIAL, `ADD}:   alu_opcode = `ALU_ADD;
            {`SPECIAL, `ADDU}:  alu_opcode = `ALU_ADDU;
            {`SPECIAL, `SUB}:   alu_opcode = `ALU_SUB;
            {`SPECIAL, `SUBU}:  alu_opcode = `ALU_SUBU;
            {`SPECIAL, `AND}:   alu_opcode = `ALU_AND;
            {`SPECIAL, `OR}:    alu_opcode = `ALU_OR;
            {`SPECIAL, `MOVN}:  alu_opcode = `ALU_PASSX;
            {`SPECIAL, `MOVZ}:  alu_opcode = `ALU_PASSX;
            {`SPECIAL, `SLT}:   alu_opcode = `ALU_SLT;
            {`SPECIAL, `SLTU}:  alu_opcode = `ALU_SLTU;
            {`SPECIAL, `SLL}:   alu_opcode = `ALU_SLL;
            {`SPECIAL, `SRL}:   alu_opcode = `ALU_SRL;
            {`SPECIAL, `SLLV}:  alu_opcode = `ALU_SLL;
            {`SPECIAL, `SRLV}:  alu_opcode = `ALU_SRL;

            
            {`SPECIAL, `XOR}:   alu_opcode = `ALU_XOR;  //ADDED BY GRAHAM
            {`XORI, `DC6}:      alu_opcode = `ALU_XOR;  //ADDED BY GRAHAM
            {`SPECIAL, `SRAV}:  alu_opcode = `ALU_SRA;  //ADDED BY GRAHAM
            {`SPECIAL, `SRA}:   alu_opcode = `ALU_SRA;  //ADDED BY GRAHAM
            {`SH, `DC6}:        alu_opcode = `ALU_ADD;  //ADDED BY GRAHAM  

            // compare rs data to 0, only care about 1 operand
            {`BGTZ, `DC6}:      alu_opcode = `ALU_PASSX;
            {`BLEZ, `DC6}:      alu_opcode = `ALU_PASSX;
            {`BLTZ_GEZ, `DC6}: begin
                if (isBranchLink)
                    alu_opcode = `ALU_PASSY; // pass link address for mem stage
                else
                    alu_opcode = `ALU_PASSX;
            end
            // pass link address to be stored in $ra
            {`JAL, `DC6}:       alu_opcode = `ALU_PASSY;
            {`SPECIAL, `JALR}:  alu_opcode = `ALU_PASSY;
            // or immediate with 0
            {`LUI, `DC6}:       alu_opcode = `ALU_PASSY;
            default:            alu_opcode = `ALU_PASSX;
    	endcase
    end

//******************************************************************************
// Compute value for 32 bit immediate data
//******************************************************************************

    //**** DON'T EDIT, THIS IS WORKING

    // all immediates are sign-extended except for:
    //  - logical instructions (andi, ori, xori)
    //  - lui, which shifts immediate into upper 16 bits

    
    wire use_imm = &{op != `SPECIAL, op != `SPECIAL2, op != `BNE, op != `BEQ}; // where to get 2nd ALU operand from: 0 for RtData, 1 for Immediate
    wire isLogicalInstr = |{(op == `ANDI), (op == `ORI), (op == `XORI)};       //ADDED BY GRAHAM
    wire isLUI = op == `LUI;                                                   //ADDED BY GRAHAM

    wire [31:0] imm_sign_extend = {{16{immediate[15]}}, immediate};
    wire [31:0] imm_zero_extend = {16'b0, immediate};                          //ADDED BY GRAHAM
    wire [31:0] imm_upper = {immediate, 16'b0};

    wire [31:0] imm = isLUI ? imm_upper :                                      //EDITED BY GRAHAM
                (isLogicalInstr ? imm_zero_extend : imm_sign_extend); 


//******************************************************************************
// forwarding and stalling logic
//******************************************************************************

    wire forward_rs_mem = &{rs_addr == reg_write_addr_mem, rs_addr != `ZERO, reg_we_mem};

    assign rs_data = forward_rs_mem ? reg_write_data_mem : rs_data_in;
    assign rt_data = rt_data_in;

    wire rs_mem_dependency = &{rs_addr == reg_write_addr_ex, mem_read_ex, rs_addr != `ZERO};

    wire read_from_rs = ~|{isLUI, jump_target, isShiftImm};

    wire isALUImm = |{op == `ADDI, op == `ADDIU, op == `SLTI, op == `SLTIU, op == `ANDI, op == `ORI};
    wire read_from_rt = ~|{isLUI, jump_target, isALUImm, mem_read};

    wire isJumpReg = isJR || isJALR;                                 //ADDED BY GRAHAM

    assign stall = (rs_mem_dependency & read_from_rs) || isJumpReg || isJ;  //EDITED BY GRAHAM

    // Forward from MEM stage if applicable, reg_write_addr_mem is from the previous instruction in the mem stage
    wire forward_rt_mem = (rt_addr == reg_write_addr_mem) && (rt_addr != `ZERO) && reg_we_mem;  //ADDED BY GRAHAM
    // Forward from EX stage if applicable 
    wire forward_rt_ex = (rt_addr == reg_write_addr_ex) && (rt_addr != `ZERO) && reg_we_ex;  //ADDED BY GRAHAM


    assign mem_halfword_ex = (op == `SH); //ADDED BY GRAHAM

    assign jr_pc = rs_data;
    
    assign mem_write_data = forward_rt_ex ? alu_result_ex :  //EDITED BY GRAHAM
                            forward_rt_mem ? reg_write_data_mem :
                            rt_data_in;

//******************************************************************************
// Determine ALU inputs and register writeback address
//******************************************************************************

    // for shift operations, use either shamt field or lower 5 bits of rs
    // otherwise use rs

    wire [31:0] shift_amount = isShiftImm ? shamt : rs_data[4:0];
    
    assign alu_op_x = (isJAL || isJALR) ? (cur_pc + 32'd8) :    //EDITED BY GRAHAM
                      isShift ? shift_amount : 
                      rs_data;

    // for link operations, use next pc (current pc + 8)
    // for immediate operations, use Imm
    // otherwise use rt

    wire [4:0] ra_addr = 5'd31;  // ADDED BY GRAHAM, $ra (return address register)

    assign alu_op_y = (use_imm) ? imm : rt_data;
    
    assign reg_write_addr = (isJAL) ? ra_addr : //EDITED BY GRAHAM
                            (isJALR) ? rd_addr :
                            (use_imm) ? rt_addr :
                            rd_addr;

    // determine when to write back to a register (any operation that isn't an
    // unconditional store, non-linking branch, or non-linking jump)
    assign reg_we = ~|{(mem_we & (op != `SC)), isJ, isJR, isBGEZNL, isBGTZ, isBLEZ, isBLTZNL, isBNE, isBEQ}; //EDITED BY GRAHAM, added isJR

    // determine whether a register write is conditional
    assign movn = &{op == `SPECIAL, funct == `MOVN};
    assign movz = &{op == `SPECIAL, funct == `MOVZ};

//******************************************************************************
// Memory control
//******************************************************************************
    assign mem_we = |{op == `SW, op == `SB, op == `SC};    // write to memory
    assign mem_read = |{op == `LB, op == `LBU, op == `LL, op == `LH, op == `LW};  //EDITED BY GRAHAM
    assign mem_byte = |{op == `SB, op == `LB, op == `LBU};    // memory operations use only one byte
    assign mem_signextend = ~|{op == `LBU};     // sign extend sub-word memory reads

//******************************************************************************
// Load linked / Store conditional
//******************************************************************************
    assign mem_sc_id = (op == `SC);

    // 'atomic_id' is high when a load-linked has not been followed by a store.
    assign atomic_id = 1'b0;

    // 'mem_sc_mask_id' is high when a store conditional should not store
    assign mem_sc_mask_id = 1'b0;

//******************************************************************************
// Branch resolution
//******************************************************************************

    wire isEqual = rs_data == rt_data;

    assign jump_branch = |{isBEQ & isEqual,
                           isBNE & ~isEqual};

    assign jump_target = isJ || isJAL; //EDITED BY GRAHAM, added isJAL
    assign jump_reg = isJumpReg;   //EDITED BY GRAHAM

endmodule
