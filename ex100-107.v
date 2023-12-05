module reg_file(
input         clk,
input  [4:0]  A1,A2,A3,
input  [31:0] WD,
input 	      WE,
output [31:0] RD1,RD2
);
reg [31:0] reg_file[0:31];
//初始化寄存器堆
integer i;
initial
begin
    for(i=0;i<32;i=i+1) reg_file[i] = 0;
end

//写入寄存器
always@(posedge clk)
begin
    if(WE)begin
        if(A3 != 0) reg_file[A3] <= WD;
    end
        
end

//读取寄存器
    assign RD1 = reg_file[A1];
    assign RD2 = reg_file[A2];

endmodule

module pc(
input              clk,
input              rst,
input              JUMP,
input       [31:0] JUMP_PC,
output reg  [31:0] pc);
wire [31:0] pc_plus4;
assign pc_plus4 = pc + 32'h4;
//计算PC
always@(posedge clk or posedge rst)
begin
	/*待填*/
    if(rst) pc <= 0;
    else begin
        if(JUMP) pc <= JUMP_PC;
        else begin
            pc <= pc_plus4;
        end
    end
end
endmodule

module imm(
input 	    [31:0] inst,
output reg	[31:0] out
);
wire	[6:0] opcode;
assign	opcode= inst[6:0];
//立即数扩展
    initial out = 32'b0;
always@(*)
begin
	case(opcode)
        7'b0010111: out[31:12] = inst[31:12];
        7'b0110111: out[31:12] = inst[31:12];
        7'b1100011:begin
            out[31:0] = {{20{inst[31]}}, inst[7], inst[30:25], inst[11:8], 1'b0};
        end
        7'b1101111:begin
            out[31:0] = {{11{inst[31]}}, inst[19:12], inst[20], inst[30:21], 1'b0};
        end
        7'b1100111:begin
            out[31:0] = {{21{inst[31]}}, inst[30:20]};
        end
        7'b0000011:begin
            out[31:0] = {{21{inst[31]}}, inst[30:20]};
        end
        7'b0100011:begin
            out[31:0] = {{21{inst[31]}}, inst[30:25], inst[11:7]};
        end
        7'b0010011:begin
            out[31:0] = {{21{inst[31]}}, inst[30:20]};
        end
        default: out = 32'b0;
	endcase
end 
endmodule

module branch(         
input [31:0]	REG1,
input [31:0] 	REG2,
input [2:0]		Type,
output     reg     BrE
);
wire signed 	[31:0] signed_REG1;
wire signed 	[31:0] signed_REG2;
//wire unsigned 	[31:0] unsigned_REG1;
//wire unsigned 	[31:0] unsigned_REG2;

assign signed_REG1 = REG1;
assign signed_REG2 = REG2; 
//assign unsigned_REG1 = REG1;
//assign unsigned_REG2 = REG2; 
always@(*)
begin
	case(Type)
        3'b010: BrE = REG1 == REG2 ? 1 : 0;
        3'b011: BrE = REG1 == REG2 ? 0 : 1;
        3'b100: BrE = signed_REG1 < signed_REG2 ? 1 : 0;
        3'b101: BrE = signed_REG1 >= signed_REG2 ? 1 : 0;
        3'b110: BrE = REG1 < REG2 ? 1 : 0;
        3'b111: BrE = REG1 >= REG2 ? 1 : 0;
        default: ;
        
    	/*待填*/  
	endcase
end
endmodule

module alu(
input [31:0] SrcA,SrcB,
input [3:0]  func,
output reg [31:0] ALUout
);

wire signed [31:0] signed_a;
wire signed [31:0] signed_b;

assign signed_a = SrcA;
assign signed_b = SrcB;
initial ALUout = 0;
always@(*)
begin
  case(func)
		/*待填*/
      4'b0000: ALUout = SrcA + SrcB;
      4'b1000: ALUout = SrcA - SrcB;
      4'b0001: ALUout = SrcA << SrcB[4:0];
      4'b0010: ALUout = signed_a < signed_b ? 1 : 0;
      4'b0011: ALUout = SrcA < SrcB ? 1 : 0;
      4'b0100: ALUout = SrcA ^ SrcB;
      4'b0101: ALUout = SrcA >> SrcB[4:0];
      4'b1101: ALUout = SrcA >>> SrcB[4:0];
      4'b0110: ALUout = SrcA | SrcB;
      4'b0111: ALUout = SrcA & SrcB;
      4'b1110: ALUout = SrcB;
      default: ALUout = 0;
      
	endcase
end 

endmodule

module mem(
input           clk,
input   [31:0]  im_addr,
output  [31:0]  im_dout,
input   [2:0]   dm_rd_ctrl,
input   [1:0]   dm_wr_ctrl,
input   [31:0]  dm_addr,
input   [31:0]  dm_din,
output reg  [31:0]  dm_dout
);

reg     [3:0]   byte_en;
reg     [31:0]  mem[0:4095];
reg     [31:0]  mem_out;
integer i;

initial
begin
    for(i=0;i<4095;i=i+1) mem[i] = 0;
end

initial
begin
  $readmemh("./problem/inst.dat",mem);
end

assign im_dout = (im_addr[31:14] == 0 && im_addr[1:0] == 0) ? mem[im_addr] : 0;

//由于不能跨单位读取数据，地址最低两位的数值决定了当前单位能读取到的数据，即mem_out
always@(*)
begin
    case(dm_addr[1:0])
    2'b00:  mem_out = mem[dm_addr[13:2]][31:0];   //pc~pc+4;
    2'b01:  mem_out = {8'h0,mem[dm_addr[13:2]][31:8]}; //pc+1~pc+4
    2'b10:  mem_out = {16'h0,mem[dm_addr[13:2]][31:16]}; //pc+2~pc+4
    2'b11:  mem_out = {24'h0,mem[dm_addr[13:2]][31:24]}; //pc+3~pc+4
    endcase
end

always@(*)
begin
    case(dm_rd_ctrl)                                         
    /*待填*/
        3'b001: dm_dout = {{24{mem_out[7]}}, mem_out[7:0]};
        3'b010: dm_dout = {24'h0, mem_out[7:0]};
        3'b011: dm_dout = {{16{mem_out[15]}}, mem_out[15:0]};
        3'b100: dm_dout = {16'h0, mem_out[15:0]};
        3'b101: dm_dout = mem_out;
        default: dm_dout = 0;
    endcase
end

always@(*)
begin
    if(dm_wr_ctrl == 2'b11)
        byte_en = 4'b1111;
    else if(dm_wr_ctrl == 2'b10)
    begin
        if(dm_addr[1] == 1'b1) 
            byte_en = 4'b1100;
        else
            byte_en = 4'b0011;
    end
    else if(dm_wr_ctrl == 2'b01)
    begin
        case(dm_addr[1:0])
        2'b00:  byte_en = 4'b0001;
        2'b01:  byte_en = 4'b0010;
        2'b10:  byte_en = 4'b0100;
        2'b11:  byte_en = 4'b1000;
        endcase
    end
    else
        byte_en = 4'b0000;
end

always@(posedge clk)
begin
    if((byte_en != 1'b0)&&(dm_addr[30:12]==19'b0))
    begin
        case(byte_en)
        	/*待填*/
            4'b1111: mem[dm_addr[13:2]] = dm_din;
            4'b1100: mem[dm_addr[13:2]][31:16] = dm_din[15:0]; 
            4'b0011: mem[dm_addr[13:2]][15:0] = dm_din[15:0];
            4'b0001: mem[dm_addr[13:2]][7:0] = dm_din[7:0];
            4'b0010: mem[dm_addr[13:2]][15:8] = dm_din[7:0];
            4'b0100: mem[dm_addr[13:2]][23:16] = dm_din[7:0];
            4'b1000: mem[dm_addr[13:2]][31:24] = dm_din[7:0];
        endcase
    end
end
endmodule


module ctrl(
input     [31:0]  inst,
output            rf_wr_en,
output reg    [1:0]   rf_wr_sel,
output            do_jump,
output reg    [2:0]   BrType,
output            alu_a_sel,
output            alu_b_sel,
output reg    [3:0]   alu_ctrl,
output reg    [2:0]   dm_rd_ctrl,
output reg    [1:0]   dm_wr_ctrl
);
wire    [6:0]   opcode;
wire    [2:0]   funct3;
wire    [6:0]   funct7;

wire    is_lui;
wire    is_auipc;
wire    is_jal;
wire    is_jalr;
wire    is_beq;
wire    is_bne;
wire    is_blt;
wire    is_bge;
wire    is_bltu;
wire    is_bgeu;
wire    is_lb;
wire    is_lh;
wire    is_lw;
wire    is_lbu;
wire    is_lhu;
wire    is_sb;
wire    is_sh;
wire    is_sw;
wire    is_addi;
wire    is_slti;
wire    is_sltiu;
wire    is_xori;
wire    is_ori;
wire    is_andi;
wire    is_slli;
wire    is_srli;
wire    is_srai;
wire    is_add;
wire    is_sub;
wire    is_sll;
wire    is_slt;
wire    is_sltu;
wire    is_xor;
wire    is_srl;
wire    is_sra;
wire    is_or;
wire    is_and;

wire    is_add_type;
wire    is_u_type;
wire    is_jump_type;
wire    is_b_type;
wire    is_r_type;
wire    is_i_type;
wire    is_s_type;

assign  opcode  = inst[6:0];
assign  funct7  = inst[31:25];
assign  funct3  = inst[14:12];

assign  is_lui  = (opcode == 7'h37) ;
assign  is_auipc= (opcode == 7'h17) ;
assign  is_jal  = (opcode == 7'h6F) ;
assign  is_jalr = (opcode == 7'h67) && (funct3 ==3'h0) ;
assign  is_beq  = (opcode == 7'h63) && (funct3 ==3'h0) ;
assign  is_bne  = (opcode == 7'h63) && (funct3 ==3'h1) ;
assign  is_blt  = (opcode == 7'h63) && (funct3 ==3'h4) ;
assign  is_bge  = (opcode == 7'h63) && (funct3 ==3'h5) ;
assign  is_bltu = (opcode == 7'h63) && (funct3 ==3'h6) ;
assign  is_bgeu = (opcode == 7'h63) && (funct3 ==3'h7) ;
assign  is_lb   = (opcode == 7'h03) && (funct3 ==3'h0) ;
assign  is_lh   = (opcode == 7'h03) && (funct3 ==3'h1) ;
assign  is_lw   = (opcode == 7'h03) && (funct3 ==3'h2) ;
assign  is_lbu  = (opcode == 7'h03) && (funct3 ==3'h4) ;
assign  is_lhu  = (opcode == 7'h03) && (funct3 ==3'h5) ;
assign  is_sb   = (opcode == 7'h23) && (funct3 ==3'h0) ;
assign  is_sh   = (opcode == 7'h23) && (funct3 ==3'h1) ;
assign  is_sw   = (opcode == 7'h23) && (funct3 ==3'h2) ;
assign  is_addi = (opcode == 7'h13) && (funct3 ==3'h0) ;
assign  is_slti = (opcode == 7'h13) && (funct3 ==3'h2) ;
assign  is_sltiu= (opcode == 7'h13) && (funct3 ==3'h3) ;
assign  is_xori = (opcode == 7'h13) && (funct3 ==3'h4) ;
assign  is_ori  = (opcode == 7'h13) && (funct3 ==3'h6) ;
assign  is_andi = (opcode == 7'h13) && (funct3 ==3'h7) ;
assign  is_slli = (opcode == 7'h13) && (funct3 ==3'h1) && (funct7 == 7'h00);
assign  is_srli = (opcode == 7'h13) && (funct3 ==3'h5) && (funct7 == 7'h00);
assign  is_srai = (opcode == 7'h13) && (funct3 ==3'h5) && (funct7 == 7'h20);
assign  is_add  = (opcode == 7'h33) && (funct3 ==3'h0) && (funct7 == 7'h00);
assign  is_sub  = (opcode == 7'h33) && (funct3 ==3'h0) && (funct7 == 7'h20);
assign  is_sll  = (opcode == 7'h33) && (funct3 ==3'h1) && (funct7 == 7'h00);
assign  is_slt  = (opcode == 7'h33) && (funct3 ==3'h2) && (funct7 == 7'h00);
assign  is_sltu = (opcode == 7'h33) && (funct3 ==3'h3) && (funct7 == 7'h00);
assign  is_xor  = (opcode == 7'h33) && (funct3 ==3'h4) && (funct7 == 7'h00);
assign  is_srl  = (opcode == 7'h33) && (funct3 ==3'h5) && (funct7 == 7'h00);
assign  is_sra  = (opcode == 7'h33) && (funct3 ==3'h5) && (funct7 == 7'h20);
assign  is_or   = (opcode == 7'h33) && (funct3 ==3'h6) && (funct7 == 7'h00);
assign  is_and  = (opcode == 7'h33) && (funct3 ==3'h7) && (funct7 == 7'h00);

assign  is_add_type = is_auipc | is_jal | is_jalr | is_b_type | is_s_type 
                    | is_lb | is_lh | is_lw | is_lbu | is_lhu | is_add | is_addi ;
assign  is_u_type   = is_lui | is_auipc ;
assign  is_jump_type= is_jal ;
assign  is_b_type   = is_beq | is_bne | is_blt | is_bge | is_bltu | is_bgeu ;
assign  is_r_type   = is_add | is_sub | is_sll | is_slt | is_sltu | is_xor 
                    | is_srl | is_sra | is_or | is_and ;
assign  is_i_type   = is_jalr | is_lb | is_lh | is_lw | is_lbu | is_lhu 
                    | is_addi | is_slti | is_sltiu | is_xori | is_ori | is_andi
                    | is_slli | is_srli | is_srai ;
assign  is_s_type   = is_sb | is_sh | is_sw ;
//rf_wr_en  
assign rf_wr_en     =  is_jump_type | is_r_type | is_u_type | is_i_type;  
  
//[1:0]rf_wr_sel
always@(*)
begin
     /*待填*/
     if(is_jalr || is_jal) rf_wr_sel = 2'b01;
     else if(is_addi || is_xori || is_ori || is_andi || is_slli || is_srli || is_srai || is_r_type || is_u_type) rf_wr_sel = 2'b10;
     else if(is_lb || is_lh || is_lw || is_lbu || is_lhu) rf_wr_sel = 2'b11;
     else rf_wr_sel = 2'b00;
end  
  
//do_jump
assign do_jump      =  is_jalr | is_jal /*待填*/ ;
  
//[2:0]BrType
always@(*)
begin
	 /*待填*/
     if(is_beq) BrType = 3'b010;
     else if(is_bne) BrType = 3'b011;
     else if(is_blt) BrType = 3'b100;
     else if(is_bge) BrType = 3'b101;
     else if(is_bltu) BrType = 3'b110;
     else if(is_bgeu) BrType = 3'b111;
     else BrType = 3'b000;
end
  
//alu_a_sel
assign alu_a_sel    =  (is_r_type | is_i_type | is_s_type) ? 1 : 0;

//alu_b_sel  
assign alu_b_sel    =  is_r_type ? 0 : 1 ;
  
//alu_ctrl
always@(*)
begin
     if(is_auipc || is_jal || is_jalr || is_b_type || is_s_type || is_lb || is_lbu || is_lh || is_lhu || is_lw || is_add || is_addi) alu_ctrl = 4'b0000;
     else if(is_sub) alu_ctrl = 4'b1000;
     else if(is_sll || is_slli) alu_ctrl = 4'b0001;
     else if(is_srl || is_srli) alu_ctrl = 4'b0101;
     else if(is_sra || is_srai) alu_ctrl = 4'b1101;
     else if(is_slt || is_slti) alu_ctrl = 4'b0010;
     else if(is_sltu || is_sltiu) alu_ctrl = 4'b0011;
     else if(is_xor || is_xori) alu_ctrl = 4'b0100;
     else if(is_or || is_ori) alu_ctrl = 4'b0110;
     else if(is_and || is_andi) alu_ctrl = 4'b0111;
     else if(is_lui) alu_ctrl = 4'b1110;
     else alu_ctrl = 4'b0000;
end
  
//[2:0]dm_rd_ctrl
always@(*)
begin
     /*待填*/
     if(is_lb) dm_rd_ctrl = 3'b001;
     else if(is_lbu) dm_rd_ctrl = 3'b010;
     else if(is_lh) dm_rd_ctrl = 3'b011;
     else if(is_lhu) dm_rd_ctrl = 3'b100;
     else if(is_lw) dm_rd_ctrl = 3'b101;
     else dm_rd_ctrl = 3'b000;
end

//[1:0]dm_wr_ctrl
always@(*)
begin
    /*待填*/
    if(is_sb) dm_wr_ctrl = 2'b01;
    else if(is_sh) dm_wr_ctrl = 2'b10;
    else if(is_sw) dm_wr_ctrl = 2'b11;
    else dm_wr_ctrl = 2'b00;
end  
endmodule


module top_module(
input clk,
input rst
);
wire        clk;
wire        rst;
wire    [31:0]  inst;

wire    [1:0]   rf_wr_sel;
reg     [31:0]  rf_wd;  
wire            rf_wr_en;
wire    [31:0]  rf_rd1,rf_rd2;
  
wire [31:0] pc;
wire [31:0] pc_plus4;
wire do_jump;
wire JUMP;
  
wire    [31:0]  imm_out;
  
wire    [2:0]   comp_ctrl;
wire		BrE;

wire            alu_a_sel;
wire            alu_b_sel;
wire    [31:0]  alu_a,alu_b,alu_out; 
wire    [3:0]   alu_ctrl;
  
wire    [2:0]   dm_rd_ctrl;
wire    [1:0]   dm_wr_ctrl;
wire    [31:0]  dm_dout;
  
always@(*)
begin
    case(rf_wr_sel)
    2'b00:  rf_wd = 32'h0;
    2'b01:  rf_wd = pc_plus4;
    2'b10:  rf_wd = alu_out;
    2'b11:  rf_wd = dm_dout;
    default:rf_wd = 32'h0;
    endcase
end
assign		pc_plus4 = pc + 32'h4;
assign		JUMP = BrE || do_jump;
assign      alu_a = alu_a_sel ? rf_rd1 : pc ;
assign      alu_b = alu_b_sel ? imm_out : rf_rd2 ;

reg_file reg_file0(
	.clk        (clk),
	.A1         (inst[19:15]),
	.A2         (inst[24:20]),
	.A3         (inst[11:7]),
	.WD         (rf_wd),
	.WE         (rf_wr_en),
	.RD1        (rf_rd1),
	.RD2        (rf_rd2)
);
pc	pc0(
	.clk        (clk),
	.rst		(rst),
    .JUMP		(JUMP),
	.JUMP_PC    (alu_out),
	.pc         (pc)
);
imm	imm0(
	.inst		(inst),
	.out    	(imm_out)
);
branch branch0(
	.REG1		(rf_rd1),
	.REG2		(rf_rd2),
	.Type		(comp_ctrl),
 	.BrE		(BrE)
);
alu alu0(
    .SrcA     	(alu_a),
	.SrcB      	(alu_b),
	.func   	(alu_ctrl),
	.ALUout    	(alu_out)
);
mem mem0(
	.clk        (clk),
	.im_addr    (pc),
	.im_dout    (inst),
	.dm_rd_ctrl (dm_rd_ctrl),
	.dm_wr_ctrl (dm_wr_ctrl),
	.dm_addr    (alu_out),
	.dm_din     (rf_rd2),  
	.dm_dout    (dm_dout)
);  //参考RISC-V架构
ctrl ctrl0(
	.inst       (inst),
	.rf_wr_en   (rf_wr_en),
	.rf_wr_sel  (rf_wr_sel),
	.do_jump    (do_jump),
	.BrType		(comp_ctrl),
	.alu_a_sel  (alu_a_sel),
	.alu_b_sel  (alu_b_sel),
	.alu_ctrl   (alu_ctrl),
	.dm_rd_ctrl (dm_rd_ctrl),
	.dm_wr_ctrl (dm_wr_ctrl)
);

endmodule